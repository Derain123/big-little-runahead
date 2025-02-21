package fetch

import org.chipsalliance.cde.config.{Config, Field, Parameters}
import freechips.rocketchip.rocket.{BuildHellaCache}

class WithHellaCachePrefetcher(hartIds: Seq[Int], p: CanInstantiatePrefetcher = MultiNextLinePrefetcherParams(handleVA=true)) extends Config((site, here, up) => {
  case BuildHellaCache => HellaCachePrefetchWrapperFactory.apply(hartIds, p, up(BuildHellaCache))
})

