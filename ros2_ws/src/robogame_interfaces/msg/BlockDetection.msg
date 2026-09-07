# One block seen by the block camera (mock or HSV backend).
string block_type          # ORANGE | PURPLE
float64 u                  # centroid pixel x
float64 v                  # centroid pixel y
float64 area               # px^2
float64 angle              # rad, block orientation in robot frame (CCW+)
float64 timestamp          # clock time of the frame
float64 distance           # m, camera-to-block estimate
