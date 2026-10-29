(function () {
  "use strict";
  const $ = (id) => document.getElementById(id),
    K = globalThis.MotionKinematics;
  try {
    const data = JSON.parse($("motion-data").textContent),
      robot = data.robot,
      frames = data.frames,
      program = data.program;
    const columns = Object.fromEntries(data.columns.map((name, index) => [name, index]));
    const value = (frame, name) => frame[columns[name]],
      count = robot.joint_names.length;
    const moving = robot.joints.filter((joint) => joint.type !== "fixed");
    const qValues = (frame) => Array.from({ length: count }, (_, axis) => value(frame, "q" + axis));
    const tcp = (frame) => ["x", "y", "z"].map((axis) => value(frame, "tcp_" + axis));
    let maxPositionError = 0,
      maxRotationError = 0;
    for (const frame of frames) {
      const geometry = K.forwardKinematics(robot, qValues(frame));
      const expected = K.poseMatrix({
        position: tcp(frame),
        orientation: ["w", "x", "y", "z"].map((axis) => value(frame, "q" + axis)),
      });
      maxPositionError = Math.max(
        maxPositionError,
        Math.hypot(...geometry.tcp.map((x, i) => x - expected[i * 4 + 3])),
      );
      for (let row = 0; row < 3; row++)
        for (let col = 0; col < 3; col++)
          maxRotationError = Math.max(
            maxRotationError,
            Math.abs(geometry.transform[row * 4 + col] - expected[row * 4 + col]),
          );
    }
    if (
      maxPositionError > 2e-8 ||
      maxRotationError > 2e-8 ||
      !Number.isFinite(maxPositionError + maxRotationError)
    )
      throw Error("3D-Kinematik stimmt nicht mit der exportierten Simulation überein.");
    document.body.dataset.fkPositionError = String(maxPositionError);
    document.body.dataset.fkRotationError = String(maxRotationError);
    const number = (x, digits = 4) =>
      Math.abs(x) > 1e6
        ? x.toExponential(3)
        : x.toLocaleString("de-DE", { minimumFractionDigits: digits, maximumFractionDigits: digits });
    const colors = { PTP: "#276ac1", LIN: "#07866d", CIRC: "#9253b4", WAIT: "#758798" };
    const state = {
      index: 0,
      axis: 0,
      playing: false,
      degrees: false,
      lastReal: 0,
      playTime: 0,
      yaw: -0.7,
      pitch: 0.37,
      distance: 2,
      target: [0, 0, 0],
      radius: 1,
    };
    let meshRenderer = null;
    let meshProblem = "";
    if (data.visual_model) {
      $("model-legal").hidden = false;
      $("model-licenses").textContent = data.legal_notices;
      try {
        meshRenderer = globalThis.MotionMeshes.create(
          $("mesh-scene"),
          data.visual_model,
          frames,
          columns,
          program.segments,
        );
      } catch (error) {
        meshProblem = "Vollständiges Modell nicht verfügbar: " + error.message + " Kinematikansicht aktiv.";
      }
    }
    function updateRenderMode() {
      const full = meshRenderer && $("render-mode").value === "mesh";
      $("mesh-scene").hidden = !full;
      document.body.dataset.renderMode = full ? "mesh" : "skeleton";
      document.body.dataset.meshLinks = String(meshRenderer ? meshRenderer.stats.links : 0);
      document.body.dataset.meshTriangles = String(meshRenderer ? meshRenderer.stats.triangles : 0);
      $("model-note").textContent =
        meshProblem ||
        (full
          ? "UR5e · 7 Visual-Baugruppen · offizielle UR-Geometrie · ohne Werkzeuganbau"
          : "Kinematikansicht: Gelenke, Verbindungslinien und TCP." +
            (data.visual_model ? "" : " Für diese Kinematik ist kein passendes Visual-Modell hinterlegt."));
    }
    if (!meshRenderer) {
      $("render-mode").value = "skeleton";
      $("render-mode").querySelector('[value="mesh"]').disabled = true;
    }
    updateRenderMode();
    $("render-mode").addEventListener("change", () => {
      updateRenderMode();
      paint();
    });
    $("mesh-scene").addEventListener("webglcontextlost", (event) => {
      event.preventDefault();
      meshRenderer = null;
      meshProblem =
        "3D-Grafikkontext unterbrochen. Kinematikansicht aktiv; zum erneuten Laden die Seite öffnen.";
      $("render-mode").value = "skeleton";
      $("render-mode").querySelector('[value="mesh"]').disabled = true;
      updateRenderMode();
      paint();
    });
    const rows = [];
    for (let axis = 0; axis < count; axis++) {
      const row = document.createElement("tr"),
        cells = [];
      for (let col = 0; col < 5; col++) {
        const cell = document.createElement("td");
        row.appendChild(cell);
        cells.push(cell);
      }
      cells[0].textContent = "J" + (axis + 1) + " · " + robot.joint_names[axis];
      cells[0].title = robot.joint_names[axis];
      row.tabIndex = 0;
      row.addEventListener("click", () => {
        state.axis = axis;
        paint();
      });
      row.addEventListener("keydown", (event) => {
        if (event.key === "Enter") {
          state.axis = axis;
          paint();
        }
      });
      $("joint-table").appendChild(row);
      const unitLabel = document.createElement("span");
      unitLabel.className = "unit";
      cells[0].appendChild(unitLabel);
      rows.push({ row, cells, unitLabel });
    }
    const segmentButtons = program.segments.map((segment) => {
      const button = document.createElement("button");
      button.type = "button";
      button.className = "segment";
      button.style.flexGrow = String(segment.end_s - segment.start_s);
      button.style.setProperty("--segment", colors[segment.operation]);
      button.style.setProperty("--wash", colors[segment.operation] + "12");
      const title = document.createElement("strong");
      title.textContent = segment.index + " · " + segment.operation;
      const duration = document.createElement("small");
      duration.textContent = number(segment.end_s - segment.start_s, 2) + " s";
      button.append(title, duration);
      button.addEventListener("click", () => {
        pause();
        setTime(segment.start_s);
      });
      $("segments").appendChild(button);
      return button;
    });
    $("subtitle").textContent =
      robot.name +
      " · " +
      count +
      " Achsen · " +
      number(program.cycle_s * 1000, 1) +
      " ms Sollwertraster · vollständig offline";
    $("program-summary").textContent =
      program.segments.length + " Schritte · " + number(program.duration_s, 2) + " s";
    $("sample-note").textContent =
      data.display_frame_count.toLocaleString("de-DE") +
      " dargestellte / " +
      data.original_frame_count.toLocaleString("de-DE") +
      " vollständig ausgewertete Samples. " +
      (data.display_frame_count < data.original_frame_count
        ? "Die Anzeige ist ausgedünnt; Segmentwechsel und Endpunkte bleiben enthalten. "
        : "") +
      "Keine Istwertmessungen.";
    $("fk-label").textContent = "ΔTCP ≤ " + maxPositionError.toExponential(2) + " m";
    $("fk-label").className = "good";
    const violations = data.sample_limit_violations.length;
    $("limit-label").textContent = violations ? "Grenzverletzungen gefunden" : "Innerhalb der Profilgrenzen";
    $("limit-label").className = violations ? "bad" : "good";
    $("scrubber").max = String(frames.length - 1);
    function frameAtTime(time) {
      let low = 0,
        high = frames.length - 1;
      while (low < high) {
        const mid = Math.ceil((low + high) / 2);
        if (value(frames[mid], "time_s") <= time) low = mid;
        else high = mid - 1;
      }
      return low;
    }
    function setTime(time) {
      state.index = frameAtTime(Math.max(0, Math.min(program.duration_s, time)));
      state.playTime = time;
      paint();
    }
    function pause() {
      state.playing = false;
      $("play").textContent = "▶ Abspielen";
    }
    $("play").addEventListener("click", () => {
      if (state.playing) {
        pause();
        return;
      }
      if (state.index === frames.length - 1) state.index = 0;
      state.playTime = value(frames[state.index], "time_s");
      state.lastReal = performance.now();
      state.playing = true;
      $("play").textContent = "❚❚ Pause";
      requestAnimationFrame(animate);
    });
    function animate(now) {
      if (!state.playing) return;
      state.playTime += (now - state.lastReal) * 0.001 * Number($("rate").value);
      state.lastReal = now;
      if (state.playTime >= program.duration_s) {
        state.playTime = program.duration_s;
        state.index = frames.length - 1;
        pause();
        paint();
        return;
      }
      state.index = frameAtTime(state.playTime);
      paint();
      requestAnimationFrame(animate);
    }
    $("previous").addEventListener("click", () => {
      pause();
      state.index = Math.max(0, state.index - 1);
      paint();
    });
    $("next").addEventListener("click", () => {
      pause();
      state.index = Math.min(frames.length - 1, state.index + 1);
      paint();
    });
    $("scrubber").addEventListener("input", () => {
      pause();
      state.index = Number($("scrubber").value);
      paint();
    });
    $("degrees").addEventListener("change", () => {
      state.degrees = $("degrees").checked;
      paint();
    });
    for (const id of ["joint-metric", "fit-limits", "tcp-group", "tcp-order", "show-path", "show-axes"])
      $(id).addEventListener("change", paint);
    document.addEventListener("keydown", (event) => {
      if (["INPUT", "SELECT", "BUTTON"].includes(event.target.tagName)) return;
      if (event.code === "Space") {
        event.preventDefault();
        $("play").click();
      }
      if (event.key === "ArrowRight") $("next").click();
      if (event.key === "ArrowLeft") $("previous").click();
    });
    function unit(axis, order) {
      return (
        (moving[axis].type === "prismatic" ? "m" : state.degrees ? "°" : "rad") +
        ["", "/s", "/s²", "/s³"][order]
      );
    }
    function axisScale(axis) {
      return state.degrees && moving[axis].type !== "prismatic" ? 180 / Math.PI : 1;
    }
    function setupCanvas(canvas) {
      const ratio = Math.min(2, window.devicePixelRatio || 1),
        rect = canvas.getBoundingClientRect();
      const width = Math.max(1, rect.width),
        height = Math.max(1, rect.height);
      if (canvas.width !== Math.round(width * ratio) || canvas.height !== Math.round(height * ratio)) {
        canvas.width = Math.round(width * ratio);
        canvas.height = Math.round(height * ratio);
      }
      const ctx = canvas.getContext("2d");
      ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
      ctx.clearRect(0, 0, width, height);
      return { ctx, width, height };
    }
    function resetCamera() {
      const points = [];
      const stride = Math.max(1, Math.ceil(frames.length / 100));
      for (let index = 0; index < frames.length; index += stride)
        points.push(...K.forwardKinematics(robot, qValues(frames[index])).points);
      const low = [0, 1, 2].map((axis) => Math.min(...points.map((point) => point[axis]))),
        high = [0, 1, 2].map((axis) => Math.max(...points.map((point) => point[axis])));
      state.target = low.map((x, i) => (x + high[i]) / 2);
      state.radius = Math.max(0.05, Math.hypot(...low.map((x, i) => high[i] - x)) / 2);
      state.distance = state.radius * 2.7;
      state.yaw = -0.7;
      state.pitch = 0.37;
    }
    $("reset-camera").addEventListener("click", () => {
      resetCamera();
      paint();
    });
    let drag = null;
    $("scene").addEventListener("pointerdown", (event) => {
      drag = { x: event.clientX, y: event.clientY };
      $("scene").setPointerCapture(event.pointerId);
    });
    $("scene").addEventListener("pointermove", (event) => {
      if (!drag) return;
      state.yaw -= (event.clientX - drag.x) * 0.008;
      state.pitch = Math.max(-1.1, Math.min(1.45, state.pitch + (event.clientY - drag.y) * 0.007));
      drag = { x: event.clientX, y: event.clientY };
      paint();
    });
    $("scene").addEventListener("pointerup", () => {
      drag = null;
    });
    $("scene").addEventListener("pointercancel", () => {
      drag = null;
    });
    $("scene").addEventListener(
      "wheel",
      (event) => {
        event.preventDefault();
        state.distance = Math.max(
          state.radius * 0.9,
          Math.min(state.radius * 15, state.distance * Math.exp(event.deltaY * 0.001)),
        );
        paint();
      },
      { passive: false },
    );
    function drawScene(frame, geometry) {
      const { ctx, width, height } = setupCanvas($("scene"));
      if (meshRenderer && $("render-mode").value === "mesh") {
        meshRenderer.render(geometry, state, width, height, $("show-path").checked, $("show-axes").checked);
        return;
      }
      const direction = [
        Math.cos(state.pitch) * Math.cos(state.yaw),
        Math.cos(state.pitch) * Math.sin(state.yaw),
        Math.sin(state.pitch),
      ];
      const right = [-Math.sin(state.yaw), Math.cos(state.yaw), 0],
        up = [
          -Math.sin(state.pitch) * Math.cos(state.yaw),
          -Math.sin(state.pitch) * Math.sin(state.yaw),
          Math.cos(state.pitch),
        ];
      const dot = (a, b) => a.reduce((sum, x, i) => sum + x * b[i], 0),
        focal = Math.min(width, height) * 1.4;
      const project = (point) => {
        const relative = point.map((x, i) => x - state.target[i]),
          depth = state.distance - dot(relative, direction);
        return [
          width / 2 + (focal * dot(relative, right)) / depth,
          height / 2 - (focal * dot(relative, up)) / depth,
          depth,
        ];
      };
      const line = (a, b, color, lineWidth = 1) => {
        const p = project(a),
          q = project(b);
        if (p[2] <= 0.01 || q[2] <= 0.01) return;
        ctx.beginPath();
        ctx.moveTo(p[0], p[1]);
        ctx.lineTo(q[0], q[1]);
        ctx.strokeStyle = color;
        ctx.lineWidth = lineWidth;
        ctx.lineCap = "round";
        ctx.stroke();
      };
      const floor = Math.min(0, ...geometry.points.map((point) => point[2])),
        gridSize = state.radius * 1.8,
        step = Math.pow(10, Math.floor(Math.log10(gridSize))) / 2;
      const boundedStep = Math.max(step, gridSize / 24);
      for (let offset = -gridSize; offset <= gridSize; offset += boundedStep) {
        line([offset, -gridSize, floor], [offset, gridSize, floor], "#cfdee6");
        line([-gridSize, offset, floor], [gridSize, offset, floor], "#cfdee6");
      }
      const axisLength = state.radius * 0.3;
      [
        ["X", [axisLength, 0, 0], "#c34342"],
        ["Y", [0, axisLength, 0], "#40894d"],
        ["Z", [0, 0, axisLength], "#4174c4"],
      ].forEach(([label, end, color]) => {
        line([0, 0, 0], end, color, 2);
        const p = project(end);
        ctx.fillStyle = color;
        ctx.font = "11px system-ui";
        ctx.fillText(label, p[0] + 5, p[1]);
      });
      if ($("show-path").checked) {
        const stride = Math.max(1, Math.ceil(frames.length / 1000));
        for (let i = stride; i < frames.length; i += stride) {
          const segment = program.segments[value(frames[i], "segment") - 1];
          line(
            tcp(frames[i - stride]),
            tcp(frames[i]),
            colors[segment.operation] + (i <= state.index ? "df" : "55"),
            i <= state.index ? 2.3 : 1.4,
          );
        }
      }
      const links = [];
      for (let i = 1; i < geometry.points.length; i++)
        links.push([geometry.points[i - 1], geometry.points[i]]);
      links.sort((a, b) => project(b[0])[2] + project(b[1])[2] - (project(a[0])[2] + project(a[1])[2]));
      links.forEach(([a, b]) => {
        line(a, b, "#b4d7e7", 13);
        line(a, b, "#286583", 7);
      });
      geometry.axes.forEach((axis) => {
        const p = project(axis.point);
        if (p[2] <= 0.01) return;
        ctx.beginPath();
        ctx.arc(p[0], p[1], axis.index === state.axis ? 7 : 5, 0, Math.PI * 2);
        ctx.fillStyle = axis.index === state.axis ? "#e79732" : "#f6fbfe";
        ctx.fill();
        ctx.strokeStyle = "#286583";
        ctx.lineWidth = 2;
        ctx.stroke();
        if ($("show-axes").checked)
          line(
            axis.point,
            axis.point.map((x, i) => x + axis.direction[i] * state.radius * 0.2),
            "#c77722",
            2,
          );
      });
      const p = project(geometry.tcp);
      ctx.beginPath();
      ctx.arc(p[0], p[1], 5, 0, Math.PI * 2);
      ctx.fillStyle = "#cf3847";
      ctx.fill();
      ctx.fillStyle = "#9c2938";
      ctx.font = "bold 11px system-ui";
      ctx.fillText("TCP", p[0] + 9, p[1] - 5);
      [
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
      ].forEach((axis, index) => {
        const direction = K.direction(geometry.transform, axis);
        line(
          geometry.tcp,
          geometry.tcp.map((x, i) => x + direction[i] * state.radius * 0.13),
          ["#c34342", "#40894d", "#4174c4"][index],
          2,
        );
      });
    }
    function drawChart(canvas, series, minimum, maximum, limits, titleUnit) {
      const { ctx, width, height } = setupCanvas(canvas),
        box = { x: 54, y: 14, w: Math.max(1, width - 70), h: height - 45 };
      const padding = Math.max((maximum - minimum) * 0.08, 1e-9);
      minimum -= padding;
      maximum += padding;
      const px = (time) => box.x + (time / program.duration_s) * box.w,
        py = (value) => box.y + box.h - ((value - minimum) / (maximum - minimum)) * box.h;
      program.segments.forEach((segment) => {
        ctx.fillStyle = colors[segment.operation] + "0d";
        ctx.fillRect(px(segment.start_s), box.y, px(segment.end_s) - px(segment.start_s), box.h);
      });
      ctx.font = "10px system-ui";
      ctx.fillStyle = "#60788b";
      ctx.lineWidth = 1;
      for (let i = 0; i <= 4; i++) {
        const y = box.y + (i * box.h) / 4,
          value = maximum - (i * (maximum - minimum)) / 4;
        ctx.strokeStyle = "#e0e8ef";
        ctx.beginPath();
        ctx.moveTo(box.x, y);
        ctx.lineTo(box.x + box.w, y);
        ctx.stroke();
        const label =
          Math.abs(value) >= 10000 || (Math.abs(value) > 0 && Math.abs(value) < 0.001)
            ? value.toExponential(1)
            : number(value, 2);
        ctx.fillText(label, 1, y + 3);
      }
      for (let i = 0; i <= 4; i++)
        ctx.fillText(
          number((program.duration_s * i) / 4, 1) + " s",
          px((program.duration_s * i) / 4) - 10,
          height - 5,
        );
      ctx.save();
      ctx.beginPath();
      ctx.rect(box.x, box.y, box.w, box.h);
      ctx.clip();
      ctx.setLineDash([5, 4]);
      ctx.strokeStyle = "#be5f60";
      limits.forEach((limit) => {
        ctx.beginPath();
        ctx.moveTo(box.x, py(limit));
        ctx.lineTo(box.x + box.w, py(limit));
        ctx.stroke();
      });
      ctx.setLineDash([]);
      ctx.strokeStyle = "#147d94";
      ctx.lineWidth = 1.7;
      ctx.beginPath();
      frames.forEach((frame, i) => {
        const x = px(value(frame, "time_s")),
          y = py(series[i]);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();
      const cursor = px(value(frames[state.index], "time_s"));
      ctx.strokeStyle = "#de9133";
      ctx.lineWidth = 1.3;
      ctx.beginPath();
      ctx.moveTo(cursor, box.y);
      ctx.lineTo(cursor, box.y + box.h);
      ctx.stroke();
      ctx.restore();
      ctx.fillStyle = "#526d82";
      ctx.fillText(titleUnit, box.x + 6, box.y + 12);
      ctx.fillText("gestrichelt: Profilgrenze", Math.max(box.x, width - 160), box.y + 12);
    }
    for (const id of ["joint-chart", "tcp-chart"])
      $(id).addEventListener("click", (event) => {
        const rect = $(id).getBoundingClientRect();
        pause();
        setTime(
          Math.max(0, Math.min(1, (event.clientX - rect.left - 54) / (rect.width - 70))) * program.duration_s,
        );
      });
    function paint() {
      const frame = frames[state.index],
        segment = program.segments[value(frame, "segment") - 1],
        geometry = K.forwardKinematics(robot, qValues(frame));
      $("scrubber").value = String(state.index);
      $("time-label").textContent =
        number(value(frame, "time_s"), 3) + " / " + number(program.duration_s, 3) + " s";
      $("tick-label").textContent = "Tick " + value(frame, "tick");
      $("step-label").textContent =
        segment.index + " · " + segment.operation + " · speed " + number(segment.speed, 2);
      $("pose-label").textContent = "TCP [m]  " + geometry.tcp.map((x) => number(x, 5)).join(" / ");
      $("operation-label").textContent =
        segment.operation + (state.index === frames.length - 1 ? " · Ende im Stillstand" : "");
      $("source-label").textContent =
        segment.source_file +
        ":" +
        segment.source_line +
        " · " +
        segment.source_block +
        " / " +
        segment.source_step;
      rows.forEach(({ row, cells, unitLabel }, axis) => {
        row.classList.toggle("selected", axis === state.axis);
        ["q", "dq", "ddq", "jerk"].forEach((prefix, order) => {
          cells[order + 1].textContent = number(value(frame, prefix + axis) * axisScale(axis), 4);
          cells[order + 1].title = unit(axis, order);
        });
        cells[0].title = robot.joint_names[axis] + " · " + unit(axis, 0);
        unitLabel.textContent = unit(axis, 0) + " · Ableitungen pro s";
      });
      segmentButtons.forEach((button, index) =>
        button.classList.toggle("active", index === segment.index - 1),
      );
      drawScene(frame, geometry);
      const metric = $("joint-metric").value,
        order = ["q", "dq", "ddq", "jerk"].indexOf(metric),
        scale = axisScale(state.axis);
      const series = frames.map((frame) => value(frame, metric + state.axis) * scale),
        capKey = { dq: "velocity", ddq: "acceleration", jerk: "jerk" }[metric];
      const limits =
        metric === "q"
          ? [robot.limits.lower[state.axis] * scale, robot.limits.upper[state.axis] * scale]
          : [-robot.limits[capKey][state.axis] * scale, robot.limits[capKey][state.axis] * scale];
      $("joint-chart-title").textContent =
        "J" + (state.axis + 1) + " · " + $("joint-metric").selectedOptions[0].textContent;
      const fitLimits = $("fit-limits").checked;
      const chartLow = Math.min(...series, ...(fitLimits ? limits : [])),
        chartHigh = Math.max(...series, ...(fitLimits ? limits : []));
      drawChart($("joint-chart"), series, chartLow, chartHigh, limits, unit(state.axis, order));
      const tcpOrder = Number($("tcp-order").value),
        group = $("tcp-group").value,
        prefix = ["tcp_v", "tcp_a", "tcp_j"][tcpOrder],
        begin = group === "linear" ? 0 : 3;
      const tcpSeries = frames.map((frame) =>
          Math.hypot(...[0, 1, 2].map((axis) => value(frame, prefix + (begin + axis)))),
        ),
        cap = robot.tcp_limits[group][tcpOrder];
      drawChart(
        $("tcp-chart"),
        tcpSeries,
        0,
        Math.max(cap, ...tcpSeries),
        [cap],
        (group === "linear" ? "m" : "rad") + ["/s", "/s²", "/s³"][tcpOrder],
      );
      document.body.dataset.currentFrame = String(state.index);
      document.body.dataset.currentTick = String(value(frame, "tick"));
      document.body.dataset.currentSegment = String(segment.index);
    }
    resetCamera();
    const initialTime = Number(new URLSearchParams(location.hash.slice(1)).get("t"));
    if (Number.isFinite(initialTime) && initialTime > 0) state.index = frameAtTime(initialTime);
    paint();
    window.addEventListener("resize", paint);
    document.body.dataset.viewerState = "ready";
  } catch (error) {
    $("error").hidden = false;
    $("error").textContent = "Darstellung gestoppt: " + error.message;
    document.body.dataset.viewerState = "error";
    $("play").disabled = true;
    console.error(error);
  }
})();
