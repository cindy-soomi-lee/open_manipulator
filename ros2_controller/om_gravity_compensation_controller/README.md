# Gravity Compensation Controller

This controller compensates gravity using KDL inverse dynamics and can apply a reflected external wrench.

## Controller telemetry

For OMY leader runs, the controller publishes nonblocking telemetry on `/omy/controller_telemetry` using `realtime_tools::RealtimePublisher`. The controller never writes telemetry files from `update()`; experiment logging should subscribe to this topic from a non-real-time process.

The telemetry contains controller period/update-compute timing, raw and KDL-scaled joint velocity, acceleration, received/used external wrench, and a torque decomposition (`tau_rne`, `tau_reflect`, `tau_spring`, `tau_sync`, `tau_friction`, `tau_pre_scale`, `tau_cmd`). `controller_seq` and `publish_missed_total` make dropped telemetry samples observable without blocking the control loop.

Quick OMY-only checks:

```bash
ros2 topic hz /omy/controller_telemetry
ros2 topic echo /omy/controller_telemetry --once
```
