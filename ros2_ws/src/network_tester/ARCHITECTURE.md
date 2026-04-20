# Network Tester Architecture

## 1. System Architecture

The `network_tester` package adopts a modular, decoupled architecture following Single Responsibility and Open-Closed principles.

- **Main Node (`node_network_tester.py`)**: The central orchestrator. Parses parameters, manages tool lifecycles (start/stop), handles ROS context, and aggregates summary data.
- **Tool Modules (`tools/*.py`)**: Isolated execution contexts for specific networking utilities (e.g., `PingTool`, `IperfTool`, `MTRTool`).
- **Plotting Module (`plot_utils.py`)**: Consumes structured metric outputs from tool modules to optionally generate visual assets using `matplotlib`.

## 2. Process Flow

```text
[1. Initialization Phase]
  ├─ NetworkTestNode parses ROS arguments
  ├─ VPNDetector determines networking environment context
  └─ Output artifacts directory is provisioned

[2. Parallel Execution Phase] (Runs for 'duration' seconds)
  ├─ PingTool (Subprocess: ping)
  ├─ IperfTool (Subprocess: iperf3)
  ├─ MTRTool (Subprocess: mtr)
  └─ ROSBandwidthMonitor (Asynchronous ROS Subscriber callback logging)

[3. Teardown & Analysis Phase]
  ├─ Subprocesses cleanly terminated
  ├─ Custom raw parsers extract metrics from stdout/json files
  ├─ Tool modules emit standard struct metrics (*.csv & *.json)
  └─ IOUtils aggregates all module metrics into meta_*.json

[4. Visualization Phase]
  └─ PlotUtils generates all timeseries charts
```

## 3. Extensibility Design

The project is structured such that adding a new analysis instrument requires zero modification to existing core utilities.

### Ex: Adding `TracerouteTool`

1. **Implement Core Logic (`tools/traceroute_tool.py`)**:
```python
class TracerouteTool:
    def __init__(self, target: str, logger=None):
        self.target = target
        self.logger = logger
    
    def start(self, output_file: str):
        # Spawn execution process logic
        pass
    
    def parse_results(self, raw_file: str) -> dict:
        # Extract and format metrics
        pass
```

2. **Export Module (`tools/__init__.py`)**:
```python
from .traceroute_tool import TracerouteTool
```

3. **Register Execution (`node_network_tester.py`)**:
Instantiate `TracerouteTool` and invoke its methods seamlessly within the initialization, execution, and parsing hooks.
