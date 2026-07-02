# Battery voltage and status
std_msgs/Header header

# Measured voltage (total pack)
float32 voltage

# Per-cell voltage (voltage / cell_count)
float32 voltage_per_cell

# Number of cells in series
uint8 cell_count

# Percentage estimate (linear between min and max voltage)
float32 percentage

# Status constants
uint8 OK=0
uint8 LOW=1
uint8 CRITICAL=2

uint8 status
