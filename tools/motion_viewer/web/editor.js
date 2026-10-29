(function () {
  "use strict";
  const $ = (id) => document.getElementById(id);
  const storageKey = "robot-motion-editor-v1";
  let config,
    busy = false,
    lastResult = null,
    lastSubmission = null,
    errorSubmission = null,
    draftTimer;
  const snapshot = () => ({
    source: $("source").value,
    filename: $("filename").value,
    cycle_ms: Number($("cycle-ms").value),
  });
  const same = (a, b) =>
    a && b && a.source === b.source && a.filename === b.filename && a.cycle_ms === b.cycle_ms;
  const localFilename = (value) => /^[a-zA-Z0-9][a-zA-Z0-9_.-]{0,74}\.ya?ml$/.test(value);

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
      $("draft-note").textContent = "Entwurf bitte als YAML speichern";
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
    busy = true;
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
      ]) {
        $(id).href = url;
        $(id).hidden = false;
        $(id).setAttribute("download", "");
      }
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
      setDraft({ source, filename: file.name, cycle_ms: Number($("cycle-ms").value) });
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
          draft.cycle_ms > 20
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
