
    var gd = document.getElementById('{plot_id}');
    var base = gd.layout.scene.camera, t = 0, spinning = true, paused = 0;
    var e0 = base.eye, up = base.up;
    var u = [up.x, up.y, up.z];
    var n = Math.hypot(u[0], u[1], u[2]); u = [u[0]/n, u[1]/n, u[2]/n];
    function rot(v, k, a) {   // Rodrigues: rotate v about unit axis k by angle a
      var c = Math.cos(a), s = Math.sin(a);
      var kd = k[0]*v[0] + k[1]*v[1] + k[2]*v[2];
      var kx = [k[1]*v[2]-k[2]*v[1], k[2]*v[0]-k[0]*v[2], k[0]*v[1]-k[1]*v[0]];
      return [v[0]*c + kx[0]*s + k[0]*kd*(1-c),
              v[1]*c + kx[1]*s + k[1]*kd*(1-c),
              v[2]*c + kx[2]*s + k[2]*kd*(1-c)];
    }
    gd.on('plotly_relayouting', function() { paused = Date.now(); });
    setInterval(function() {
      if (!spinning || Date.now() - paused < 2500) return;
      t += 0.012;
      var a = 0.315 * Math.sin(t);                       // +/- 18 degrees
      var e = rot([e0.x, e0.y, e0.z], u, a);
      Plotly.relayout(gd, {'scene.camera.eye': {x: e[0], y: e[1], z: e[2]}});
    }, 60);
    var b = document.createElement('button');
    b.textContent = 'pause rotation';
    b.style.cssText = 'position:absolute;top:8px;right:16px;z-index:9;font:13px sans-serif;' +
                      'padding:4px 10px;border:1px solid #ccc;border-radius:5px;background:#fff;' +
                      'cursor:pointer';
    b.onclick = function() {
      spinning = !spinning;
      b.textContent = spinning ? 'pause rotation' : 'resume rotation';
    };
    document.body.appendChild(b);
    