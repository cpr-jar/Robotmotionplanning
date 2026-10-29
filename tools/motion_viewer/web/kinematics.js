/* Viewer mathematics only. The library supplies admitted geometry and commands. */
(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  else root.MotionKinematics = api;
})(typeof globalThis !== "undefined" ? globalThis : this, function () {
  "use strict";
  function identity() {
    return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
  }
  function multiply(a, b) {
    const c = Array(16).fill(0);
    for (let row = 0; row < 4; row++)
      for (let col = 0; col < 4; col++)
        for (let k = 0; k < 4; k++) c[row * 4 + col] += a[row * 4 + k] * b[k * 4 + col];
    return c;
  }
  function poseMatrix(pose) {
    const norm = Math.hypot(...pose.orientation);
    if (!Number.isFinite(norm) || norm === 0) throw Error("Ungültiges Quaternion");
    const [w, x, y, z] = pose.orientation.map((v) => v / norm),
      [px, py, pz] = pose.position;
    return [
      1 - 2 * (y * y + z * z),
      2 * (x * y - z * w),
      2 * (x * z + y * w),
      px,
      2 * (x * y + z * w),
      1 - 2 * (x * x + z * z),
      2 * (y * z - x * w),
      py,
      2 * (x * z - y * w),
      2 * (y * z + x * w),
      1 - 2 * (x * x + y * y),
      pz,
      0,
      0,
      0,
      1,
    ];
  }
  function jointMotion(type, axis, value) {
    const [x, y, z] = axis,
      transform = identity();
    if (type === "prismatic") {
      transform[3] = x * value;
      transform[7] = y * value;
      transform[11] = z * value;
    } else if (type === "revolute" || type === "continuous") {
      const c = Math.cos(value),
        s = Math.sin(value),
        d = 1 - c;
      transform.splice(
        0,
        12,
        c + x * x * d,
        x * y * d - z * s,
        x * z * d + y * s,
        0,
        y * x * d + z * s,
        c + y * y * d,
        y * z * d - x * s,
        0,
        z * x * d - y * s,
        z * y * d + x * s,
        c + z * z * d,
        0,
      );
    } else if (type !== "fixed") throw Error("Unbekannter Gelenktyp");
    return transform;
  }
  function position(matrix) {
    return [matrix[3], matrix[7], matrix[11]];
  }
  function direction(matrix, vector) {
    return [0, 1, 2].map(
      (row) =>
        matrix[row * 4] * vector[0] + matrix[row * 4 + 1] * vector[1] + matrix[row * 4 + 2] * vector[2],
    );
  }
  function forwardKinematics(robot, jointValues) {
    if (jointValues.length !== robot.joint_names.length || !jointValues.every(Number.isFinite))
      throw Error("Ungültiger Gelenkvektor");
    let transform = poseMatrix(robot.base_pose),
      movingIndex = 0;
    const points = [position(transform)],
      axes = [],
      linkFrames = Object.create(null);
    if (robot.base_link) linkFrames[robot.base_link] = transform.slice();
    for (const joint of robot.joints) {
      transform = multiply(transform, poseMatrix(joint.origin));
      points.push(position(transform));
      if (joint.type !== "fixed") {
        axes.push({
          point: position(transform),
          direction: direction(transform, joint.axis),
          index: movingIndex,
        });
        transform = multiply(transform, jointMotion(joint.type, joint.axis, jointValues[movingIndex++]));
        if (joint.type === "prismatic") points.push(position(transform));
      }
      if (joint.child_link) linkFrames[joint.child_link] = transform.slice();
    }
    if (movingIndex !== jointValues.length) throw Error("Gelenkreihenfolge stimmt nicht");
    transform = multiply(transform, poseMatrix(robot.tool_offset));
    points.push(position(transform));
    return { points, axes, transform, tcp: position(transform), linkFrames };
  }
  return { identity, multiply, poseMatrix, forwardKinematics, direction };
});
