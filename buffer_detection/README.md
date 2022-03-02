# Notes 
Target - Efficient buffer place sampling (requires fast rendering of objects, less number of renderings during optimal buffer place search)

## Time taken
1. Time taken to render 2-2-2: 9.5 sec
2. Time taken for rendering a scene (on an average): around 8 seconds (estimate)
3. Time taken to find a buffer spot (naive): 0.0123 seconds
4. Time taken to find closest buffer via marching-grid: 7.4265 seconds (for 9 steps (single cycle))