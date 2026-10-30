(function () {
  "use strict";
  const $ = (id) => document.getElementById(id);
  const storageKey = "robot-motion-editor-v1";
  let config,
    busy = false,
    activeJobId = null,
    lastResult = null,
    lastSubmission = null,
    errorSubmission = null,
    draftTimer,
    assets = [];
  const snapshot = () => ({
    source: $("source").value,
    filename: $("filename").value,
    cycle_ms: Number($("cycle-ms").value),
    assets: assets.map((asset) => ({ ...asset })),
  });
  const same = (a, b) =>
    a &&
    b &&
    a.source === b.source &&
    a.filename === b.filename &&
    a.cycle_ms === b.cycle_ms &&
    (a.assets || []).length === (b.assets || []).length &&
    (a.assets || []).every(
      (asset, index) =>
        asset.filename === b.assets[index].filename && asset.content === b.assets[index].content,
    );
  const localFilename = (value) => /^[a-zA-Z0-9][a-zA-Z0-9_.-]{0,74}\.ya?ml$/.test(value);
  const dxfFilename = (value) => /^[a-zA-Z0-9][a-zA-Z0-9_.-]{0,75}\.dxf$/i.test(value);
  function validAssets(values) {
    return (
      Array.isArray(values) &&
      values.length <= config.max_assets &&
      values.every(
        (asset) =>
          asset &&
          typeof asset.filename === "string" &&
          dxfFilename(asset.filename) &&
          typeof asset.content === "string" &&
          asset.content.trim() &&
          !asset.content.includes("\0") &&
          new TextEncoder().encode(asset.content).length <= config.max_asset_bytes,
      ) &&
      new Set(values.map((asset) => asset.filename.toLowerCase())).size === values.length
    );
  }
  function renderAssets() {
    const previous = $("dxf-file").value;
    $("dxf-file").replaceChildren();
    for (const asset of assets) {
      const option = document.createElement("option");
      option.value = option.textContent = asset.filename;
      $("dxf-file").appendChild(option);
    }
    if (assets.some((asset) => asset.filename === previous)) $("dxf-file").value = previous;
    $("insert-dxf").disabled = $("remove-dxf").disabled = assets.length === 0;
    $("dxf-note").textContent = assets.length
      ? assets.length + " DXF-Datei(en) geladen. Einfügen ergänzt einen Befehl; Simulieren plant die Bahn."
      : "Eine Text-DXF laden oder das DXF-Beispiel wählen. Der Befehl wird am Ende des steps-Blocks ergänzt.";
    document.body.dataset.dxfFiles = String(assets.length);
  }

  async function request(url, options) {
    const response = await fetch(url, { cache: "no-store", ...options });
    const result = await response.json();
    if (!response.ok) {
      const error = new Error(result.error?.message || "Anfrage fehlgeschlagen.");
      error.diagnostic = result.error;
      throw error;
    }
    return result;
  }
  function updateLines() {
    const lines = $("source").value.split("\n");
    $("line-numbers").textContent = lines.map((_, index) => index + 1).join("\n");
    $("line-numbers").scrollTop = $("source").scrollTop;
    const before = $("source").value.slice(0, $("source").selectionStart).split("\n");
    $("cursor").textContent = "Zeile " + before.length + " · Spalte " + (before.at(-1).length + 1);
  }
  function updateResultState() {
    const current = lastSubmission && same(snapshot(), lastSubmission);
    document.body.dataset.resultState = lastResult ? (current ? "current" : "stale") : "none";
    $("result-state").textContent = lastResult
      ? current
        ? "Aktueller Programmstand"
        : "Älterer Programmstand"
      : "Noch kein Ergebnis";
    $("go-to-error").hidden =
      !errorSubmission || !same(snapshot(), errorSubmission.request) || !errorSubmission.line;
    $("diagnostic-title").textContent =
      errorSubmission && !same(snapshot(), errorSubmission.request)
        ? "Fehler im zuvor simulierten Stand"
        : "Programm nicht simuliert";
  }
  function persistDraft() {
    try {
      localStorage.setItem(storageKey, JSON.stringify(snapshot()));
      $("draft-note").textContent = "Entwurf im Browser gespeichert";
    } catch (_) {
      $("draft-note").textContent = "Entwurf bitte sichern; DXF-Dateien zusätzlich aufbewahren";
    }
  }
  function changed() {
    updateLines();
    updateResultState();
    clearTimeout(draftTimer);
    draftTimer = setTimeout(persistDraft, 250);
    if (!busy)
      $("run-status").textContent =
        lastResult && same(snapshot(), lastSubmission)
          ? "Der angezeigte Lauf gehört genau zu diesem Programmstand."
          : "Programm bereit – mit Simulieren vollständig prüfen und ausführen.";
  }
  function setDraft(draft) {
    assets = (draft.assets || []).map((asset) => ({ ...asset }));
    renderAssets();
    $("source").value = draft.source;
    $("filename").value = draft.filename;
    $("cycle-ms").value = String(draft.cycle_ms ?? 10);
    $("source").scrollTop = 0;
    $("source").setSelectionRange(0, 0);
    changed();
  }
  function download(text, filename) {
    const url = URL.createObjectURL(new Blob([text], { type: "text/yaml;charset=utf-8" }));
    const link = document.createElement("a");
    link.href = url;
    link.download = filename;
    document.body.appendChild(link);
    link.click();
    link.remove();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  }
  function showError(error, submitted) {
    $("diagnostic").hidden = false;
    const detail = error.diagnostic || { message: error.message };
    const location = detail.line
      ? "Zeile " +
        detail.line +
        (detail.column ? ", Spalte " + detail.column : "") +
        (detail.step ? " · Schritt " + detail.step : "") +
        "\n"
      : "";
    $("diagnostic-text").textContent = location + detail.message;
    errorSubmission = submitted
      ? { request: submitted, line: detail.line, column: detail.column || 1 }
      : null;
    updateResultState();
  }
  async function simulate() {
    if (busy || !config) return;
    const submitted = snapshot();
    if (!localFilename(submitted.filename)) {
      showError(new Error("Bitte einen einfachen Dateinamen mit Endung .yaml oder .yml verwenden."));
      return;
    }
    if (
      !submitted.source.trim() ||
      new TextEncoder().encode(submitted.source).length > config.max_source_bytes
    ) {
      showError(new Error("Bitte ein Programm mit höchstens 1 MiB eingeben."));
      return;
    }
    if (!Number.isInteger(submitted.cycle_ms) || submitted.cycle_ms < 1 || submitted.cycle_ms > 20) {
      showError(new Error("Das Sollwertraster muss zwischen 1 und 20 ms liegen."));
      return;
    }
    submitted.job_id = crypto.randomUUID().replaceAll("-", "");
    activeJobId = submitted.job_id;
    busy = true;
    $("cancel-simulation").disabled = false;
    document.body.dataset.editorState = "simulating";
    $("simulate").disabled = true;
    $("simulate").textContent = "Planung & Simulation …";
    $("diagnostic").hidden = true;
    errorSubmission = null;
    $("run-status").textContent =
      "C++-Planer und virtuelle Ausführung laufen … Zeitbudget: " + config.timeout_s + " s.";
    persistDraft();
    try {
      const result = await request("/api/simulate", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          "X-Motion-Token": config.token,
        },
        body: JSON.stringify(submitted),
      });
      lastResult = result;
      lastSubmission = submitted;
      $("preview-placeholder").hidden = true;
      $("preview").hidden = false;
      $("preview").src = result.preview_url;
      $("result-summary").textContent =
        result.robot +
        " · " +
        result.segments +
        " Schritte · " +
        result.duration_s.toLocaleString("de-DE", { maximumFractionDigits: 3 }) +
        " s · " +
        result.commands.toLocaleString("de-DE") +
        " Befehle · " +
        result.cycle_ms +
        " ms";
      for (const [id, url] of [
        ["download-html", result.html_url],
        ["download-snapshot", result.source_url],
        ["download-bundle", result.bundle_url],
      ]) {
        $(id).href = url;
        $(id).hidden = false;
        $(id).setAttribute("download", "");
      }
      $("download-bundle").hidden = !result.dxf_files;
      $("run-status").textContent = same(snapshot(), submitted)
        ? "Simulation erfolgreich. Im Viewer Abspielen drücken."
        : "Simulation erfolgreich. Der Entwurf wurde währenddessen geändert – für diesen Stand erneut simulieren.";
    } catch (error) {
      showError(error, submitted);
      $("run-status").textContent =
        "Keine neue Ansicht freigegeben." +
        (lastResult ? " Die letzte gültige Simulation bleibt sichtbar." : " Bitte das Programm korrigieren.");
    } finally {
      busy = false;
      activeJobId = null;
      $("cancel-simulation").disabled = true;
      $("simulate").disabled = false;
      $("simulate").textContent = "▶ Simulieren";
      document.body.dataset.editorState = "ready";
      updateResultState();
    }
  }
  for (const id of ["source", "filename", "cycle-ms"]) $(id).addEventListener("input", changed);
  $("source").addEventListener("scroll", () => {
    $("line-numbers").scrollTop = $("source").scrollTop;
  });
  for (const event of ["click", "keyup", "select"]) $("source").addEventListener(event, updateLines);
  $("source").addEventListener("keydown", (event) => {
    if (event.key === "Tab") {
      event.preventDefault();
      $("source").setRangeText("  ", $("source").selectionStart, $("source").selectionEnd, "end");
      changed();
    }
  });
  document.addEventListener("keydown", (event) => {
    if ((event.ctrlKey || event.metaKey) && ["Enter", "s"].includes(event.key)) {
      event.preventDefault();
      if (event.key === "Enter") simulate();
      else $("save").click();
    }
  });
  $("simulate").addEventListener("click", simulate);
  $("cancel-simulation").addEventListener("click", async () => {
    if (!activeJobId) return;
    const jobId = activeJobId;
    $("cancel-simulation").disabled = true;
    try {
      const result = await request("/api/cancel", {
        method: "POST",
        headers: {"Content-Type": "application/json", "X-Motion-Token": config.token},
        body: JSON.stringify({job_id: jobId}),
      });
      if (activeJobId === jobId) {
        if (result.cancelled) $("run-status").textContent = "Abbruch angefordert …";
        else $("cancel-simulation").disabled = false;
      }
    } catch (error) {
      if (activeJobId === jobId) {
        $("cancel-simulation").disabled = false;
        $("run-status").textContent = "Abbruchanfrage fehlgeschlagen: " + error.message;
      }
    }
  });
  $("open-dxf").addEventListener("click", () => $("dxf-import").click());
  $("dxf-import").addEventListener("change", async () => {
    try {
      if (!config) throw Error("Editor wird noch geladen.");
      const next = assets.map((asset) => ({ ...asset }));
      for (const file of $("dxf-import").files) {
        if (!dxfFilename(file.name) || file.size > config.max_asset_bytes)
          throw Error("DXF-Dateien: einfacher Dateiname ohne Pfad, UTF-8-Text, maximal 1 MiB pro Datei.");
        const content = new TextDecoder("utf-8", { fatal: true }).decode(await file.arrayBuffer());
        const asset = { filename: file.name, content };
        const previous = next.findIndex((item) => item.filename === file.name);
        if (previous >= 0) next[previous] = asset;
        else next.push(asset);
      }
      if (!validAssets(next))
        throw Error("Maximal vier eindeutige DXF-Dateien mit Textinhalt laden (keine binäre DXF).");
      assets = next;
      renderAssets();
      changed();
    } catch (error) {
      showError(error);
    } finally {
      $("dxf-import").value = "";
    }
  });
  $("remove-dxf").addEventListener("click", () => {
    assets = assets.filter((asset) => asset.filename !== $("dxf-file").value);
    renderAssets();
    changed();
  });
  $("insert-dxf").addEventListener("click", () => {
    try {
      const source = $("source").value,
        lines = source.split("\n");
      const start = lines.findIndex((line) => /^steps:\s*(?:#.*)?$/.test(line));
      if (start < 0 || !$("dxf-file").value)
        throw Error("Zum Einfügen muss steps: auf einer eigenen Zeile stehen und eine DXF geladen sein.");
      let end = start + 1;
      while (end < lines.length && !/^[^\s#-][^:]*:/.test(lines[end])) end++;
      const item = lines.slice(start + 1, end).find((line) => /^\s*-/.test(line));
      const indent = item ? item.match(/^\s*/)[0] : "  ";
      const detail = indent + "  ";
      const command = [
        indent + "- type: DXF",
        detail + "file: " + JSON.stringify($("dxf-file").value),
        detail + "units: " + $("dxf-units").value,
        detail + "reference: " + $("dxf-reference").value,
        detail + "travel: " + $("dxf-travel").value,
        detail + "speed: 0.5",
      ];
      if ($("dxf-layer").value.trim())
        command.push(detail + "layer: " + JSON.stringify($("dxf-layer").value.trim()));
      if ($("dxf-reference").value === "world")
        command.push(detail + "placement: {position: [0,0,0], orientation: [1,0,0,0]}");
      const before = lines.slice(0, end).join("\n");
      const offset = before.length + (before.endsWith("\n") ? 0 : 1);
      $("source").value =
        before +
        (before.endsWith("\n") ? "" : "\n") +
        command.join("\n") +
        "\n" +
        lines.slice(end).join("\n");
      $("source").focus();
      $("source").setSelectionRange(offset, offset);
      changed();
    } catch (error) {
      showError(error);
    }
  });
  $("save").addEventListener("click", () => {
    const draft = snapshot();
    if (!localFilename(draft.filename)) {
      showError(new Error("Bitte einen einfachen YAML-Dateinamen verwenden."));
      return;
    }
    download(draft.source, draft.filename);
    persistDraft();
  });
  $("load-example").addEventListener("click", async () => {
    try {
      setDraft(await request("/api/examples/" + encodeURIComponent($("example").value)));
    } catch (error) {
      showError(error);
    }
  });
  $("import").addEventListener("click", () => $("import-file").click());
  $("import-file").addEventListener("change", async () => {
    const file = $("import-file").files[0];
    if (!file) return;
    try {
      if (!config || file.size > config.max_source_bytes)
        throw Error("Programmdatei ist zu groß (maximal 1 MiB).");
      if (!localFilename(file.name)) throw Error("Bitte eine YAML-Datei mit einfachem Dateinamen wählen.");
      const source = new TextDecoder("utf-8", { fatal: true }).decode(await file.arrayBuffer());
      setDraft({ source, filename: file.name, cycle_ms: Number($("cycle-ms").value), assets });
    } catch (error) {
      showError(error);
    } finally {
      $("import-file").value = "";
    }
  });
  $("go-to-error").addEventListener("click", () => {
    if (!errorSubmission || !same(snapshot(), errorSubmission.request)) return;
    const lines = $("source").value.split("\n");
    const line = Math.max(0, Math.min(lines.length - 1, errorSubmission.line - 1));
    const offset = lines.slice(0, line).reduce((sum, value) => sum + value.length + 1, 0);
    $("source").focus();
    $("source").setSelectionRange(offset, offset + lines[line].length);
    $("source").scrollTop = Math.max(0, line * parseFloat(getComputedStyle($("source")).lineHeight) - 80);
    updateLines();
  });
  async function init() {
    try {
      config = await request("/api/config");
      for (const example of config.examples) {
        const option = document.createElement("option");
        option.value = example.id;
        option.textContent = example.label;
        $("example").appendChild(option);
      }
      let draft;
      try {
        draft = JSON.parse(localStorage.getItem(storageKey));
        if (
          !draft ||
          typeof draft.source !== "string" ||
          new TextEncoder().encode(draft.source).length > config.max_source_bytes ||
          !localFilename(draft.filename) ||
          !Number.isInteger(draft.cycle_ms) ||
          draft.cycle_ms < 1 ||
          draft.cycle_ms > 20 ||
          !validAssets(draft.assets || [])
        )
          draft = null;
      } catch (_) {
        draft = null;
      }
      setDraft(draft || (await request("/api/examples/ur5e")));
      $("run-status").textContent = draft
        ? "Lokaler Entwurf wiederhergestellt. Zum Prüfen Simulieren drücken."
        : "UR5e-Beispiel geladen. Du kannst es bearbeiten oder direkt simulieren.";
      $("simulate").disabled = false;
      document.body.dataset.editorState = "ready";
    } catch (error) {
      showError(
        new Error(
          "Lokaler Editor nicht erreichbar. Mit run_editor.py starten und diese Seite neu laden. " +
            error.message,
        ),
      );
      document.body.dataset.editorState = "error";
    }
  }
  init();
})();
