# Aircraft Autopilot Project

This project was done for the class ASE 370C: Feedback Control Systems in the Spring 2026 semester.

## Summary

The project objective of improving upon the baseline attitude-hold controller has been successfully achieved through the development and implementation of a cascaded flight control architecture.

The baseline controller provided basic attitude stabilization using direct feedback but lacked the ability to regulate full flight conditions or maintain performance under disturbance. In contrast, the improved controller introduces a structured approach consisting of:

- An outer loop regulating altitude, heading, and airspeed  
- An inner loop stabilizing attitude with rate and integral feedback  

This cascaded structure allows high-level flight objectives to be translated into physically meaningful control actions.

As a result, the improved controller demonstrates:

- Significantly improved disturbance rejection  
- Reduced oscillatory behavior through rate feedback  
- Improved steady-state accuracy via integral action  
- More stable and consistent tracking of flight conditions  

In simulation, the system is capable of maintaining straight-and-level flight under turbulent conditions, representing a clear performance improvement over the baseline controller in both transient response and robustness.

Additionally, telemetry streaming and real-time visualization tools have been integrated, enabling:

- External monitoring of system states  
- Data-driven tuning and validation  
- Improved debugging and performance analysis  

Overall, the improved controller meets the intended design goals and represents a substantial advancement over the baseline implementation.
