"use strict";
const fs = require("fs"),
  assert = require("assert");
const K = require("../web/kinematics.js");
let cases = 0,
  maxPosition = 0,
  maxRotation = 0;
for (const filename of process.argv.slice(2)) {
  const data = JSON.parse(fs.readFileSync(filename, "utf8"));
  const lines = data.trace_csv.trim().split("\n"),
    columns = Object.fromEntries(
      lines
        .shift()
        .split(",")
        .map((name, index) => [name, index]),
    );
  for (const line of lines) {
    const row = line.split(",").map(Number),
      get = (name) => row[columns[name]],
      q = data.robot.joint_names.map((_, axis) => get("q" + axis));
    const rendered = K.forwardKinematics(data.robot, q);
    const expected = K.poseMatrix({
      position: ["x", "y", "z"].map((axis) => get("tcp_" + axis)),
      orientation: ["w", "x", "y", "z"].map((axis) => get("q" + axis)),
    });
    const position = Math.hypot(...rendered.tcp.map((value, axis) => value - expected[axis * 4 + 3]));
    assert(Number.isFinite(position));
    maxPosition = Math.max(maxPosition, position);
    for (let r = 0; r < 3; r++)
      for (let c = 0; c < 3; c++) {
        const error = Math.abs(rendered.transform[r * 4 + c] - expected[r * 4 + c]);
        assert(Number.isFinite(error));
        maxRotation = Math.max(maxRotation, error);
      }
    assert(maxPosition < 2e-8 && maxRotation < 2e-8);
    cases++;
  }
}
console.log(JSON.stringify({ cases, maxPosition, maxRotation }));
