"""mock — complete mock environment (Mac / CI, no hardware).

Wires the REAL software stack around a simulated vehicle:
  McuClient <-> MemoryTransport <-> FakeSTM32 <-> SimWorld
Everything above the Transport is the code that runs on the Pi.
"""
