import argparse
import m5
from m5.objects import *

# ==============================================================================
# 1. Cache Hierarchy Definitions (L1I, L1D, L2, Shared L3)
# ==============================================================================
class L1ICache(Cache):
    size = '32kB'
    assoc = 4
    tag_latency = 1
    data_latency = 1
    response_latency = 1
    mshrs = 8
    tgts_per_mshr = 8
    # Critical-word-first is enabled by default in gem5 Classic Caches

class L1DCache(Cache):
    size = '32kB'
    assoc = 4
    tag_latency = 1      # Models MEM1 translation + tag lookup
    data_latency = 1     # Models MEM2 data formatting/return
    response_latency = 1
    mshrs = 8
    tgts_per_mshr = 8
    prefetcher = StridePrefetcher(degree=4, latency=1)

class L2Cache(Cache):
    size = '512kB'
    assoc = 8
    tag_latency = 6
    data_latency = 6
    response_latency = 2
    mshrs = 16
    tgts_per_mshr = 12
    prefetcher = StridePrefetcher(degree=8, latency=1)

class L3CacheSlice(Cache):
    size = '4MB'
    assoc = 16
    tag_latency = 18
    data_latency = 18
    response_latency = 4
    mshrs = 16
    tgts_per_mshr = 16

# ==============================================================================
# 2. ARM O3 Core Definition
# ==============================================================================
class CloudArmO3CPU(ArmO3CPU):
    """
    Approximates pipeline timing using O3 delays, not eight distinct stages.
    """
    fetchToDecodeDelay = 1
    decodeToRenameDelay = 1
    renameToIEWDelay = 2      # Accounts for RR + EX1 dispatch
    iewToCommitDelay = 2      # Accounts for EX2/MEM2 + WB
    
    # Pipeline widths (balanced for energy-efficient cloud throughput)
    fetchWidth = 4
    decodeWidth = 4
    renameWidth = 4
    issueWidth = 4
    wbWidth = 4
    commitWidth = 4

    # Queue sizes
    numROBEntries = 96
    instQueues = [IQUnit(numEntries=48)]
    LQEntries = 32
    SQEntries = 32


class CpuPowerModel(PowerModel):
    def __init__(
        self,
        subsystem,
        alpha_c_pf,
        frequency_hz,
        voltage_v,
        static_on_mw, # Leakage current
        static_gated_mw, # Gated leakage current
        static_off_mw, # OFF state leakage current
    ):
        super().__init__(subsystem=subsystem)
        dynamic_on_w = alpha_c_pf * 1e-12 * frequency_hz * voltage_v**2
        self.pm = [
            MathExprPowerModel(dyn=str(dynamic_on_w), st=str(static_on_mw / 1000)),
            MathExprPowerModel(dyn="0", st=str(static_gated_mw / 1000)),
            MathExprPowerModel(dyn="0", st=str(static_off_mw / 1000)),
            MathExprPowerModel(dyn="0", st=str(static_off_mw / 1000)),
        ]

# ==============================================================================
# 3. System Assembly Function
# ==============================================================================
def build_system(binary_path, num_cores=16, perf_level=0, power_gating=True):
    system = System()
    system.cpu_power_subsystem = SubSystem()

    # Global clock & voltage domain for interconnect and memory
    system.voltage_domain = VoltageDomain(voltage='1.0V')
    system.clk_domain = SrcClockDomain(
        clock='2.0GHz',
        voltage_domain=system.voltage_domain
    )

    system.mem_mode = 'timing'
    system.mem_ranges = [AddrRange('4GB')]
    system.cache_line_size = 64

    # One shared operating point for all cores, fixed for the duration of a run.
    system.cpu_voltage_domain = VoltageDomain(voltage=['1.1V', '0.95V', '0.8V'])
    system.cpu_clk_domain = SrcClockDomain(
        clock=['2.5GHz', '1.8GHz', '1.2GHz'],
        voltage_domain=system.cpu_voltage_domain,
        init_perf_level=perf_level,
    )

    # gem5 gates the clock immediately on thread suspension, not after 100
    # cycles. OFF is reached after more than 1000 suspended CPU cycles.
    system.cpu = [
        CloudArmO3CPU(
            cpu_id=i,
            clk_domain=system.cpu_clk_domain,
            power_gating_on_idle=power_gating,
            pwr_gating_latency=1001,
        )
        for i in range(num_cores)
    ]

    # Shared L2-to-L3 Coherent Crossbar
    system.tol3bus = CoherentXBar(
        width=64,
        frontend_latency=2,
        forward_latency=2,
        response_latency=2,
        snoop_response_latency=2
    )

    # Connect Private L1I, L1D, and L2 Caches to Each Core
    for cpu in system.cpu:
        cpu.icache = L1ICache()
        cpu.dcache = L1DCache()
        cpu.l2cache = L2Cache()

        # Connect L1s to CPU
        cpu.icache.cpu_side = cpu.icache_port
        cpu.dcache.cpu_side = cpu.dcache_port

        # Private L1-to-L2 Bus
        cpu.l2bus = L2XBar()
        cpu.icache.mem_side = cpu.l2bus.cpu_side_ports
        cpu.dcache.mem_side = cpu.l2bus.cpu_side_ports

        # Connect L2 to L2Bus and Shared L3Bus
        cpu.l2cache.cpu_side = cpu.l2bus.mem_side_ports
        cpu.l2cache.mem_side = system.tol3bus.cpu_side_ports

        # Create Interrupt Controller (Required for ARM)
        cpu.createInterruptController()

    # Four shared slices total 16MB. Bits 18-19 select a slice, above the
    # 4MB, 16-way cache's set-index bits, so every set remains reachable.
    system.l3_slices = [
        L3CacheSlice(
            addr_ranges=[
                AddrRange(
                    start=mem_range.start,
                    end=mem_range.end,
                    intlvHighBit=19,
                    intlvBits=2,
                    intlvMatch=slice_id,
                )
                for mem_range in system.mem_ranges
            ]
        )
        for slice_id in range(4)
    ]
    system.membus = SystemXBar()

    for l3_slice in system.l3_slices:
        l3_slice.cpu_side = system.tol3bus.mem_side_ports
        l3_slice.mem_side = system.membus.cpu_side_ports

    # System Port for Binary Loading & Functional Access
    system.system_port = system.membus.cpu_side_ports

    # DDR4 Main Memory Controller
    system.mem_ctrl = MemCtrl()
    system.mem_ctrl.dram = DDR4_2400_16x4()
    system.mem_ctrl.dram.range = system.mem_ranges[0]
    system.mem_ctrl.port = system.membus.mem_side_ports

    # Set up Syscall Emulation (SE) Workload for Testing
    system.workload = SEWorkload.init_compatible(binary_path)
    
    # Assign the binary to Core 0 (or pass a multi-threaded binary)
    process = Process()
    process.cmd = [binary_path]
    for cpu in system.cpu:
        cpu.workload = process
        cpu.createThreads()

    return system

# ==============================================================================
# 4. Simulation Execution Entry Point
# ==============================================================================
if __name__ == "__m5_main__":
    parser = argparse.ArgumentParser(description="16-Core Energy-Efficient ARM Simulation")
    parser.add_argument("--binary", type=str, required=True, help="Path to AArch64 binary")
    parser.add_argument(
        "--num-cores",
        type=int,
        default=16,
        help="Number of simulated CPU cores",
    )
    parser.add_argument(
        "--perf-level",
        type=int,
        choices=(0, 1, 2),
        default=0,
        help="Fixed CPU operating point: 0=2.5GHz/1.1V, "
        "1=1.8GHz/0.95V, 2=1.2GHz/0.8V",
    )
    parser.add_argument(
        "--no-power-gating",
        action="store_true",
        help="Keep suspended cores clock-gated instead of entering OFF",
    )
    parser.add_argument(
        "--alpha-c-pf",
        type=float,
        default=100.0,
        help="Effective alpha*C in pF (default is illustrative, not calibrated)",
    )
    parser.add_argument(
        "--static-on-mw",
        type=float,
        default=100.0,
        help="ON-state static power per core in mW (illustrative default)",
    )
    parser.add_argument(
        "--static-gated-mw",
        type=float,
        default=50.0,
        help="CLK_GATED-state static power per core in mW (illustrative default)",
    )
    parser.add_argument(
        "--static-off-mw",
        type=float,
        default=10.0,
        help="OFF-state static power per core in mW (illustrative default)",
    )
    args = parser.parse_args()

    power_parameters = (
        args.alpha_c_pf,
        args.static_on_mw,
        args.static_gated_mw,
        args.static_off_mw,
    )
    if any(value < 0 for value in power_parameters):
        parser.error("power model parameters must be non-negative")

    system = build_system(
        args.binary,
        num_cores=args.num_cores,
        perf_level=args.perf_level,
        power_gating=not args.no_power_gating,
    )

    frequency_hz = (2.5e9, 1.8e9, 1.2e9)[args.perf_level]
    voltage_v = (1.1, 0.95, 0.8)[args.perf_level]
    for cpu in system.cpu:
        cpu.power_state.default_state = "ON"
        cpu.power_model = CpuPowerModel(
            subsystem=system.cpu_power_subsystem,
            alpha_c_pf=args.alpha_c_pf,
            frequency_hz=frequency_hz,
            voltage_v=voltage_v,
            static_on_mw=args.static_on_mw,
            static_gated_mw=args.static_gated_mw,
            static_off_mw=args.static_off_mw,
        )

    root = Root(full_system=False, system=system)
    m5.instantiate()

    print(
        "WARNING: CPU power model defaults are illustrative, not calibrated. "
        "Use measured alpha*C and leakage values for quantitative results."
    )
    print(f"Starting {args.num_cores}-core ARM simulation...")
    exit_event = m5.simulate()
    print(f"Exiting @ tick {m5.curTick()} because {exit_event.getCause()}")
