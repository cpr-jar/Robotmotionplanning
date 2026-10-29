/* Offline visual geometry renderer. All dimensions use metres and ROS Z-up. */
(function (root) {
  "use strict";
  const K = root.MotionKinematics;

  function unpack(encoded, integer = false) {
    const raw = atob(encoded);
    if (raw.length % 4) throw Error("Ungültiger Mesh-Puffer.");
    const bytes = Uint8Array.from(raw, (value) => value.charCodeAt(0));
    const view = new DataView(bytes.buffer);
    const result = integer ? new Uint32Array(raw.length / 4) : new Float32Array(raw.length / 4);
    for (let i = 0; i < result.length; i++) {
      result[i] = integer ? view.getUint32(i * 4, true) : view.getFloat32(i * 4, true);
      if (!Number.isFinite(result[i])) throw Error("Nicht endliche Mesh-Koordinate.");
    }
    return result;
  }

  function create(canvas, model, frames, columns, segments) {
    const T = root.THREE;
    if (!T) throw Error("Offline-3D-Renderer fehlt.");
    const renderer = new T.WebGLRenderer({ canvas, antialias: true, alpha: true });
    renderer.setPixelRatio(Math.min(2, root.devicePixelRatio || 1));
    renderer.setClearColor(0x000000, 0);
    renderer.outputColorSpace = T.SRGBColorSpace;
    const scene = new T.Scene();
    const camera = new T.PerspectiveCamera(45, 1, 0.001, 100);
    camera.up.set(0, 0, 1);
    scene.add(new T.HemisphereLight(0xffffff, 0x657582, 2));
    const key = new T.DirectionalLight(0xffffff, 2.1);
    key.position.set(2, -3, 5);
    scene.add(key);
    const fill = new T.DirectionalLight(0xffffff, 0.8);
    fill.position.set(-3, 2, 1);
    scene.add(fill);
    const links = new Map();
    const stats = { links: 0, triangles: 0, parts: 0, linkMatrices: {} };
    const color = (rgba) => new T.Color(rgba[0], rgba[1], rgba[2]).convertSRGBToLinear();
    try {
      for (const visual of model.links) {
        const group = new T.Group();
        group.matrixAutoUpdate = false;
        for (const part of visual.parts) {
          const positions = unpack(part.positions);
          const indices = unpack(part.indices, true);
          const normals = unpack(part.normals);
          if (
            positions.length !== part.vertex_count * 3 ||
            indices.length !== part.index_count ||
            indices.some((index) => index >= part.vertex_count) ||
            (part.primitive === "triangles" && normals.length !== positions.length)
          )
            throw Error("Ungültige Mesh-Geometrie: " + visual.link);
          const geometry = new T.BufferGeometry();
          geometry.setAttribute("position", new T.BufferAttribute(positions, 3));
          geometry.setIndex(new T.BufferAttribute(indices, 1));
          let object;
          if (part.primitive === "lines") {
            object = new T.LineSegments(geometry, new T.LineBasicMaterial({ color: color(part.color) }));
          } else if (part.primitive === "triangles") {
            geometry.setAttribute("normal", new T.BufferAttribute(normals, 3));
            object = new T.Mesh(
              geometry,
              new T.MeshPhongMaterial({
                color: color(part.color),
                shininess: 45,
                specular: 0x555555,
                opacity: part.color[3],
                transparent: part.color[3] < 1,
                side: T.DoubleSide,
              }),
            );
            stats.triangles += indices.length / 3;
          } else throw Error("Unbekannte Mesh-Primitive.");
          group.add(object);
          stats.parts++;
        }
        scene.add(group);
        links.set(visual.link, { group, origin: K.poseMatrix(visual.origin) });
      }
      stats.links = links.size;
      if (stats.triangles !== model.triangle_count) throw Error("Unvollständiges Visual-Modell.");
    } catch (error) {
      dispose();
      throw error;
    }

    const grid = new T.GridHelper(1, 24, 0xb5cbd8, 0xcfdee6);
    grid.rotation.x = Math.PI / 2;
    scene.add(grid);
    const worldAxes = new T.AxesHelper(1);
    scene.add(worldAxes);
    const toolAxes = new T.AxesHelper(1);
    toolAxes.matrixAutoUpdate = false;
    scene.add(toolAxes);
    const tip = new T.Mesh(new T.SphereGeometry(1, 16, 12), new T.MeshBasicMaterial({ color: 0xcf3847 }));
    scene.add(tip);
    const arrows = [];
    const pathPositions = [],
      pathColors = [];
    const operationColors = { PTP: 0x276ac1, LIN: 0x07866d, CIRC: 0x9253b4, WAIT: 0x758798 };
    for (let i = 1; i < frames.length; i++) {
      const tint = new T.Color(operationColors[segments[frames[i][columns.segment] - 1].operation]);
      for (const index of [i - 1, i]) {
        for (const axis of ["x", "y", "z"]) pathPositions.push(frames[index][columns["tcp_" + axis]]);
        pathColors.push(tint.r, tint.g, tint.b);
      }
    }
    function pathLine(opacity) {
      const geometry = new T.BufferGeometry();
      geometry.setAttribute("position", new T.Float32BufferAttribute(pathPositions, 3));
      geometry.setAttribute("color", new T.Float32BufferAttribute(pathColors, 3));
      const line = new T.LineSegments(
        geometry,
        new T.LineBasicMaterial({
          vertexColors: true,
          transparent: opacity < 1,
          opacity,
          depthWrite: opacity === 1,
        }),
      );
      scene.add(line);
      return line;
    }
    const future = pathLine(0.22),
      past = pathLine(1);

    function render(kinematics, state, width, height, showPath, showAxes) {
      for (const [name, visual] of links) {
        const frame = kinematics.linkFrames[name];
        if (!frame) throw Error("Link-Transformation fehlt: " + name);
        const matrix = K.multiply(frame, visual.origin);
        visual.group.matrix.fromArray(matrix).transpose();
      }
      const focal = Math.min(width, height) * 1.4;
      camera.fov = (2 * Math.atan(height / (2 * focal)) * 180) / Math.PI;
      camera.aspect = width / height;
      camera.near = Math.max(0.0001, state.radius * 0.001);
      camera.far = Math.max(100, state.distance + state.radius * 10);
      camera.updateProjectionMatrix();
      camera.position.set(
        state.target[0] + state.distance * Math.cos(state.pitch) * Math.cos(state.yaw),
        state.target[1] + state.distance * Math.cos(state.pitch) * Math.sin(state.yaw),
        state.target[2] + state.distance * Math.sin(state.pitch),
      );
      camera.lookAt(...state.target);
      renderer.setSize(width, height, false);
      grid.scale.setScalar(state.radius * 3.6);
      grid.position.z = Math.min(0, ...kinematics.points.map((point) => point[2]));
      worldAxes.scale.setScalar(state.radius * 0.3);
      toolAxes.matrix.fromArray(kinematics.transform).transpose();
      toolAxes.matrix.scale(new T.Vector3(...Array(3).fill(state.radius * 0.13)));
      tip.position.set(...kinematics.tcp);
      tip.scale.setScalar(state.radius * 0.012);
      future.visible = past.visible = showPath;
      past.geometry.setDrawRange(0, state.index * 2);
      for (let i = 0; i < kinematics.axes.length; i++) {
        const axis = kinematics.axes[i];
        if (!arrows[i]) {
          arrows[i] = new T.ArrowHelper(new T.Vector3(0, 0, 1), new T.Vector3(), 1, 0xc77722);
          scene.add(arrows[i]);
        }
        arrows[i].visible = showAxes;
        arrows[i].position.set(...axis.point);
        arrows[i].setDirection(new T.Vector3(...axis.direction));
        arrows[i].setLength(state.radius * 0.2, state.radius * 0.04, state.radius * 0.02);
      }
      renderer.render(scene, camera);
      for (const [name, visual] of links) stats.linkMatrices[name] = visual.group.matrixWorld.toArray();
    }

    function dispose() {
      scene.traverse((object) => {
        if (object.geometry) object.geometry.dispose();
        if (object.material) object.material.dispose();
      });
      renderer.dispose();
    }
    return { render, dispose, stats };
  }
  root.MotionMeshes = { create };
})(globalThis);
