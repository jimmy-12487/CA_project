#=========================================================================
# riscvi2oi Subpackage
#=========================================================================

riscvi2oi_deps = \
  vc \
  imuldiv \

riscvi2oi_srcs = \
  riscvi2oi-CoreDpath.v \
  riscvi2oi-CoreDpathRegfile.v \
  riscvi2oi-CoreDpathAlu.v \
  riscvi2oi-CoreDpathPipeMulDiv.v \
  riscvi2oi-CoreCtrl.v \
  riscvi2oi-Core.v \
  riscvi2oi-InstMsg.v \

riscvi2oi_test_srcs = \
  riscvi2oi-InstMsg.t.v \
  riscvi2oi-CoreDpathPipeMulDiv.t.v \

riscvi2oi_prog_srcs = \
  riscvi2oi-sim.v \
  riscvi2oi-randdelay-sim.v \

