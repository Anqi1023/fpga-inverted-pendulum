create_clock -name sys_clk -period 37.037037 [get_ports {clk}]
// External human inputs are asynchronous; no invented source-clock delays.
// First-stage CDC/async-reset paths and external output budgets require review.
// This file intentionally does not hide these paths with broad false paths.
