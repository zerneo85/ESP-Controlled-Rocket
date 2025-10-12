// -----------------------------
// Helpers
// -----------------------------
const $ = (sel, root = document) => root.querySelector(sel);

function bytesToKB(n) { return (n / 1024).toFixed(2); }
function bytesToMB(n) { return (n / 1024 / 1024).toFixed(2); }

function flash(msg, type = "info") {
  // Optional: show messages in a banner if you have #flash
  const el = $("#flash");
  if (!el) { alert(msg); return; }
  el.textContent = msg;
  el.className = `flash ${type}`;
  setTimeout(() => { el.textContent = ""; el.className = "flash"; }, 3000);
}

// -----------------------------
// File listing & rendering
// -----------------------------
async function fetchFileLists() {
  const res = await fetch("/listfiles");
  if (!res.ok) throw new Error(`List error: HTTP ${res.status}`);
  return res.json();
}

function renderLists(data) {
  const sdUl = $("#sd-list");
  const spiffsUl = $("#spiffs-list");
  if (!sdUl || !spiffsUl) return;

  // SD list (MB)
  sdUl.innerHTML = "";
  data.sd.forEach(f => {
    const fname = f.name.startsWith("/") ? f.name : "/" + f.name;
    const li = document.createElement("li");
    li.innerHTML = `
      <code>${fname}</code> – ${bytesToMB(f.size)} MB
      <button class="btn-del" data-fs="SD" data-name="${encodeURIComponent(fname)}">Delete</button>
      <button class="btn-dl" data-fs="SD" data-name="${encodeURIComponent(fname)}">Download</button>

    `;
    sdUl.appendChild(li);
  });

  // SPIFFS list (KB)
  spiffsUl.innerHTML = "";
  data.spiffs.forEach(f => {
    const fname = f.name.startsWith("/") ? f.name : "/" + f.name;
    const li = document.createElement("li");
    li.innerHTML = `
      <code>${fname}</code> – ${bytesToKB(f.size)} KB
      <button class="btn-del" data-fs="SPIFFS" data-name="${encodeURIComponent(fname)}">Delete</button>
      <button class="btn-dl" data-fs="SPIFFS" data-name="${encodeURIComponent(fname)}">Download</button>
    `;
    spiffsUl.appendChild(li);
  });
}

async function refreshLists() {
  try {
    const data = await fetchFileLists();
    renderLists(data);
  } catch (e) {
    flash(e.message || "Failed to load file lists", "error");
  }
}

// -----------------------------
// Single file delete (SD or SPIFFS)
// -----------------------------
async function deleteFile(filename, fsType) {
  const decoded = decodeURIComponent(filename);
  if (!confirm(`Delete ${decoded}?`)) return;

  const endpoint = (fsType === "SPIFFS") ? "/deleteFileSPIFFS" : "/deleteFileSD";
  const res = await fetch(`${endpoint}?name=${filename}`);
  if (res.ok) {
    flash("File deleted.", "ok");
    await refreshLists();
  } else {
    const t = await res.text().catch(() => "");
    flash(`Deletion failed: HTTP ${res.status} ${t}`, "error");
  }
}

// -----------------------------
// Delete ALL on SD (POST)
// -----------------------------
async function deleteAllSD() {
  if (!confirm("This will delete ALL files and folders on the SD card. Are you absolutely sure?")) return;

  // POST to match the firmware route (also safest)
  let res;
  try {
    res = await fetch("/deleteAllSD", { method: "POST" });
  } catch (e) {
    flash("Delete-all failed (network): " + e.message, "error");
    return;
  }

  let body = "";
  try { body = await res.text(); } catch (_) { }

  if (res.ok || res.status === 207) {
    // 200 = clean sweep; 207 = some failures (we still refresh)
    try {
      const stats = JSON.parse(body);
      const summary =
        `Removed files: ${stats.removed_files}, ` +
        `Removed dirs: ${stats.removed_dirs}, ` +
        `Failed: ${stats.failed}`;
      flash(`SD cleanup done (HTTP ${res.status}). ${summary}`, stats.failed ? "warn" : "ok");
    } catch {
      flash(`SD cleanup done (HTTP ${res.status}).`, "ok");
    }
    await refreshLists();
  } else {
    flash(`Delete-all failed: HTTP ${res.status} ${body}`, "error");
  }
}

// -----------------------------
// Event wiring
// -----------------------------
window.addEventListener("DOMContentLoaded", async () => {
  // Initial load
  await refreshLists();

  // Delegate clicks in SD list
  $("#sd-list")?.addEventListener("click", e => {
    const btn = e.target.closest("button");
    if (!btn) return;

    if (btn.classList.contains("btn-del")) {
      deleteFile(btn.dataset.name, btn.dataset.fs);
    } else if (btn.classList.contains("btn-dl")) {
      const name = btn.dataset.name;
      const fs = btn.dataset.fs || (e.currentTarget.id === "sd-list" ? "SD" : "SPIFFS");
      const ep = (fs === "SD") ? "/downloadFileSD" : "/downloadFileSPIFFS";
      window.open(`${ep}?name=${name}`, "_blank");
    }
  });

  // Delegate clicks in SPIFFS list
  $("#spiffs-list")?.addEventListener("click", e => {
    const btn = e.target.closest("button");
    if (!btn) return;

    if (btn.classList.contains("btn-del")) {
      deleteFile(btn.dataset.name, btn.dataset.fs);
    } else if (btn.classList.contains("btn-dl")) {
      const name = btn.dataset.name;
      const fs = btn.dataset.fs || (e.currentTarget.id === "sd-list" ? "SD" : "SPIFFS");
      const ep = (fs === "SD") ? "/downloadFileSD" : "/downloadFileSPIFFS";
      window.open(`${ep}?name=${name}`, "_blank");
    }
  });

  // Delete-all button (if present)
  $("#btn-delete-all-sd")?.addEventListener("click", deleteAllSD);
});

// ===== FILE MANAGER SCRIPT (served at /files/script.js) =====

// Fetch and render file lists on page load
window.addEventListener("DOMContentLoaded", () => {
  fetch("/listfiles")
    .then(r => r.json())
    .then(data => {
      // Render SD card files (sizes in MB, always with leading slash)
      const sdUl = document.getElementById("sd-list");
      sdUl.innerHTML = "";
      data.sd.forEach(f => {
        const fname = f.name.startsWith("/") ? f.name : "/" + f.name;
        sdUl.innerHTML += `<li>${fname} - ${(f.size / 1024 / 1024).toFixed(2)} MB 
          <button onclick="deleteFile('${fname}', 'SD')">Delete</button>
          <button onclick="window.open('/downloadFileSD?name=${encodeURIComponent(fname)}','_blank')">Download</button>
        </li>`;
      });

      // Render SPIFFS files (sizes in KB, always with leading slash)
      const spiffsUl = document.getElementById("spiffs-list");
      spiffsUl.innerHTML = "";
      data.spiffs.forEach(f => {
        const fname = f.name.startsWith("/") ? f.name : "/" + f.name;
        spiffsUl.innerHTML += `<li>${fname} - ${(f.size / 1024).toFixed(2)} KB 
          <button onclick="deleteFile('${fname}', 'SPIFFS')">Delete</button>
<button onclick="window.open('/downloadFileSPIFFS?name=${encodeURIComponent(fname)}','_blank')">Download</button>
        </li>`;
      });
    })
    .catch(err => {
      alert("Failed to load file lists: " + err.message);
    });
});

// Handles file deletion via API call
function deleteFile(filename, fsType) {
  if (!confirm('Delete ' + filename + '?')) return;
  const endpoint = (fsType === 'SPIFFS') ? '/deleteFileSPIFFS' : '/deleteFileSD';
  fetch(endpoint + '?name=' + encodeURIComponent(filename))
    .then(res => {
      if (res.ok) {
        alert('File deleted.');
        location.reload();
      } else {
        return res.text().then(t => { throw new Error(t || res.statusText); });
      }
    })
    .catch(err => alert('Deletion failed: ' + err.message));
}

// BIG red “Delete ALL” button — uses POST per your server.on("/deleteAllSD", HTTP_POST, ...)
function deleteAllSD() {
  if (!confirm("This will delete ALL files and folders on the SD card. Are you absolutely sure?")) return;
  fetch("/deleteAllSD", { method: "POST" })
    .then(async res => {
      const text = await res.text();
      if (!res.ok) throw new Error(text || "Deletion failed");
      alert("All SD files deleted.\n" + text);
      location.reload();
    })
    .catch(err => alert("Failed: " + err.message));
}

// Expose for inline onclick
window.deleteAllSD = deleteAllSD;
window.deleteFile = deleteFile;
