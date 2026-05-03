
//  MAIN  Tab switching, Chart.js loader, init


(function () {
  // Loading Chart.js from CDN then init all modules
  function loadScript(src, cb) {
    const s = document.createElement('script');
    s.src = src;
    s.onload = cb;
    document.head.appendChild(s);
  }

  loadScript(
    'https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.1/chart.umd.js',
    function () {
      // Set Chart.js global defaults
      Chart.defaults.color = '#4a5a4a';
      Chart.defaults.borderColor = 'rgba(0,255,136,0.08)';
      Chart.defaults.font.family = 'Space Mono';

      PID.init();
      StateSpace.init();
      Trajectory.init();

      // Tab switching
      document.querySelectorAll('.tab').forEach(tab => {
        tab.addEventListener('click', () => {
          const target = tab.dataset.tab;
          document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
          document.querySelectorAll('.tab-panel').forEach(p => p.classList.remove('active'));
          tab.classList.add('active');
          document.getElementById('tab-' + target).classList.add('active');
          // Triggering resize so canvases re-render
          setTimeout(() => window.dispatchEvent(new Event('resize')), 50);
        });
      });
    }
  );
})();
