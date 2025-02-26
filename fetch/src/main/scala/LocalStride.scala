package fetch

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Field, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._
import freechips.rocketchip.tile._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.subsystem.{CacheBlockBytes}

case class LocalStridedPrefetcherParams(
  nSets: Int = 128,
  ahead: Int = 4
) extends CanInstantiatePrefetcher{
  def desc() = "Local Strided Prefetcher"
  def instantiate()(implicit p: Parameters) = Module(new LocalStridedPrefetcher(this)(p))
}

object StrideStates {
  val width = 2
  def Init    = 0.U(width.W)
  def Steady  = 1.U(width.W)
  def Trans   = 2.U(width.W)
  def NoPred  = 3.U(width.W)
}

class StrideMetadata extends Bundle {
  val state = UInt(StrideStates.width.W)
    /** Metadata equality */
  def ===(rhs: UInt): Bool = state === rhs
  def ===(rhs: StrideMetadata): Bool = state === rhs.state
  def =/=(rhs: StrideMetadata): Bool = !this.===(rhs)

  def isValid(dummy: Int = 0): Bool = state > StrideStates.Init

  private def StateTrans(corr: Bool): (Bool, UInt) = {
    import StrideStates._
    MuxTLookup(Cat(corr, state), (false.B, 0.U(width.W)),
      Seq(
        Cat(true.B , Init)    -> (true.B,  Steady),
        Cat(false.B, Init)    -> (false.B, Trans),
        Cat(true.B , Steady)  -> (true.B,  Steady),
        Cat(false.B, Steady)  -> (false.B, Init),
        Cat(true.B , Trans)   -> (true.B,  Steady),
        Cat(false.B, Trans)   -> (false.B, NoPred),
        Cat(true.B , NoPred)  -> (false.B, Trans),
        Cat(false.B, NoPred)  -> (false.B, NoPred)
      )
    )
  }

  def onPref(corr: Bool): (Bool, StrideMetadata) = {
    val r = StateTrans(corr)
    (r._1, StrideMetadata(r._2))
  }

}

object StrideMetadata {
  def apply(perm: UInt) = {
    val meta = Wire(new StrideMetadata)
    meta.state := perm
    meta
  }
  def onReset = StrideMetadata(StrideStates.Init)
}


class LocalStridedPrefetcher(params: LocalStridedPrefetcherParams)(implicit p: Parameters) extends AbstractPrefetcher()(p) {

  class RPTMeta extends CoreBundle {
    val state = new StrideMetadata
    val tag   = UInt((vaddrBitsExtended - log2Ceil(params.nSets)).W)
    val prev_addr = UInt(coreMaxAddrBits.W)
  }

  class RPTStirde extends CoreBundle {
    val stride = UInt(coreMaxAddrBits.W)
  } 

  val _meta = Wire(new RPTMeta)
  val _stride = Wire(new RPTStirde)

  val prefetch = Reg(UInt())

  val metaSz = _meta.asUInt.getWidth
  val dataSz = _stride.asUInt.getWidth

  val meta    = SyncReadMem(params.nSets, UInt(metaSz.W))
  val delta   = SyncReadMem(params.nSets, UInt(dataSz.W))

  _meta := DontCare
  _stride := DontCare


  // --------------------------------------------------------
  // **** Stage 0 ****
  //      Send request to PRT
  // --------------------------------------------------------
  val idSz = log2Ceil(params.nSets)
  val block_bits = log2Up(io.request.bits.blockBytes)
  val ahead_bits = log2Up(params.ahead)

  val s0_valid = io.snoop.valid
  val s0_idx = io.snoop.bits.pc(idSz-1,0)
  val s0_addr = io.snoop.bits.address
  val s0_tag = io.snoop.bits.pc >> idSz

  // --------------------------------------------------------
  // **** Stage 1 ****
  //      RPT response and start prefetching
  // --------------------------------------------------------
  val s1_addr = RegEnable(s0_addr, s0_valid)
  val s1_tag = RegEnable(s0_tag, s0_valid)
  val s1_idx = RegEnable(s0_idx, s0_valid)
  val s1_valid = RegNext(s0_valid)

  val s1_req_rprt = delta.read(s0_idx, s0_valid).asTypeOf(new RPTStirde)
  val s1_req_rmeta = meta.read(s0_idx, s0_valid).asTypeOf(new RPTMeta)

  val s1_rprt = RegEnable(s1_req_rprt, s1_valid)
  val s1_rmeta = RegEnable(s1_req_rmeta, s1_valid)

  val s1_stride = s1_rprt.stride
  val s1_hit = s1_rmeta.tag === s1_tag
  val s1_prev_addr = s1_rmeta.prev_addr
  val s1_hit_state = Mux(s1_hit, s1_rmeta.state.asUInt, 0.U).asTypeOf(chiselTypeOf(StrideMetadata.onReset))
  
  val s1_new_stride = RegNext(s1_addr) - s1_prev_addr
  val s1_stirde_pos = s1_addr > s1_prev_addr
  val s1_stride_corr = s1_new_stride === s1_stride
  val (s1_can_prf, s1_new_state) = s1_hit_state.onPref(s1_stride_corr & s1_hit)
  val s1_update = s1_hit_state =/= s1_new_state || s1_prev_addr =/= s1_addr

  // prefetch controller
  val s_idle :: s_active :: Nil = Enum(2)
  val state = RegInit(s_idle)

  val pref_far_enough = s1_stride >= (1.U << block_bits.U)

  io.request.valid := state === s_active
  io.request.bits.address := prefetch
  io.request.bits.write := DontCare

  when(state === s_idle && s1_can_prf) {
    //Begin prefetching
    state := s_active
    prefetch := s1_addr + Mux(pref_far_enough, s1_stride, s1_stride << ahead_bits)
  }

  when(io.request.fire) {
    state := s_idle
  }
  // --------------------------------------------------------
  // **** Stage 2 ****
  //      Update or Allocate PRT 
  // --------------------------------------------------------
  val s2_update_meta = Wire(new RPTMeta)
  s2_update_meta.prev_addr := s1_addr
  s2_update_meta.state := Mux(s1_hit, s1_new_state.asUInt, 0.U).asTypeOf(chiselTypeOf(StrideMetadata.onReset))
  s2_update_meta.tag := s1_tag
  
  val s2_update_stride = Mux(s1_hit, s1_new_stride, 0.U)

  when(s1_valid && (s1_hit && s1_update || !s1_hit)) {
    meta.write(s1_idx, s2_update_meta.asUInt)
  }

  when(s1_valid && (s1_hit && !s1_stride_corr || !s1_hit)) {
    delta.write(s1_idx, s2_update_stride)
  }

}