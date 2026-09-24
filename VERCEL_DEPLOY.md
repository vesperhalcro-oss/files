# Deploy the browser replay to Vercel

1. Push the contents of this repository to GitHub.
2. Import the repository into Vercel.
3. Select **Other** as the framework preset.
4. Leave the build command and output directory empty.
5. Deploy.

The browser demo bundles its replay frames in `demo/replay-data.js`, so it does
not request JSON files at runtime. The demo is recorded LiDAR replay data, not
live physical LiDAR input.
