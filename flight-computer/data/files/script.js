// Fetch and render file lists on page load
window.addEventListener("DOMContentLoaded", () => {
  fetch("/listfiles")
    .then(r => r.json())
    .then(data => {
      // Render SD card files (sizes in MB, always with leading slash)
      let sdUl = document.getElementById("sd-list");
      sdUl.innerHTML = "";
      data.sd.forEach(f => {
        let fname = f.name.startsWith("/") ? f.name : "/" + f.name;
        sdUl.innerHTML += `<li>${fname} - ${(f.size/1024/1024).toFixed(2)} MB 
        <button onclick="deleteFile('${fname}', 'SD')">Delete</button>
        <button onclick="window.open('/downloadFile?name=${encodeURIComponent(fname)}','_blank')">Download</button>
        </li>`;
      });

      // Render SPIFFS files (sizes in KB, always with leading slash)
      let spiffsUl = document.getElementById("spiffs-list");
      spiffsUl.innerHTML = "";
      data.spiffs.forEach(f => {
        let fname = f.name.startsWith("/") ? f.name : "/" + f.name;
        spiffsUl.innerHTML += `<li>${fname} - ${(f.size/1024).toFixed(2)} KB 
        <button onclick="deleteFile('${fname}', 'SPIFFS')">Delete</button>
        <button onclick="window.open('/downloadFile?name=${encodeURIComponent(fname)}','_blank')">Download</button>
        </li>`;
      });
    });
});

// Handles file deletion via API call
function deleteFile(filename, fsType) {
  if (confirm('Are you sure you want to delete ' + filename + '?')) {
    var endpoint = (fsType === 'SPIFFS') ? '/deleteFileSPIFFS' : '/deleteFileSD';
    fetch(endpoint + '?name=' + encodeURIComponent(filename))
      .then(res => {
        if(res.ok) {
          alert('File deleted successfully!');
          location.reload();
        } else {
          alert('Deletion failed: ' + res.statusText);
        }
      });
  }
}
