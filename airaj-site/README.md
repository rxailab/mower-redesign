# AIRAJ website (R1 robot lawn mower)

Static site for the AIRAJ brand, built around the original (unmodified) mower design. English, UK market.

- `index.html`, `assets/css/site.css`, `assets/js/site.js`; images in `assets/img` (made with `scripts/airaj/`, see its README).
- Colour cards are generated in `site.js` from the `COLOURS` list (same colours as `viewer/original/models/palettes.json`).
- Help Me Choose uses the design targets (about 130 m2 per hour, 90 minutes per charge, slopes up to 35 %, lawns up to 1,000 m2).
- The register form is not connected to any sign-up service yet: it validates and shows a notice, nothing is sent or stored.
- Preview locally: `python3 -m http.server 8790 --directory airaj-site`.
