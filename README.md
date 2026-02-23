This project simulates a SystemVerilog “tiny GPU” using:
- sv2v (SystemVerilog → Verilog conversion)
- Icarus Verilog (iverilog/vvp) as the simulator
- cocotb (Python-based testbench runner)
- A Python virtual environment (venv) to keep dependencies isolated
- A Makefile with targets like make test_matadd, make test_matmul, etc.

Steps to simulate:
- source venv/bin/activate
- make test_matadd
- make test_matmul
