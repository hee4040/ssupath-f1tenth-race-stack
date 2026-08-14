# Output compatibility

This editor release keeps the version-0.1/v3 generation contract unchanged.

Verified during packaging:

- `src/f1tenth_raceline/core.py` is byte-identical to the previous v3 bundle.
- The output-writing section of `cmd_generate()` is AST-identical to v3.
- An empty edit profile materializes an occupancy image whose decoded pixels are exactly equal to the original image.
- The temporary edited YAML preserves the original `resolution` and `origin` values and only points `image` to the temporary edited image.
- Local editor API tests passed for state load, centerline preview, profile save, and shutdown.

Therefore:

- Without `--edit`, computation and outputs use the pre-editor path.
- With an empty `--edit`, planner inputs are equivalent to the original map.
- With actual edits, numeric trajectory values may change by design, but filenames, CSV columns, default optimizer settings, and `summary.json` schema remain unchanged.


## TPH 0.80 / SciPy compatibility patch (v0.2.2)

`trajectory-planning-helpers==0.80` still contains this pattern in
`spline_approximation.dist_to_p()`:

```python
s = interpolate.splev(t_glob, path)
return spatial.distance.euclidean(p, s)
```

`scipy.optimize.fmin()` supplies the one optimization variable as a `(1,)`
NumPy array. Modern SciPy then sees the parametric spline point as `(2, 1)`
and rejects it because `euclidean()` requires 1-D vectors.

v0.2.2 installs a runtime shim that converts the single spline parameter to a
Python scalar before calling the same `splev()` and the same Euclidean
distance. The planner core, optimizer configuration, output filenames, CSV
columns, and JSON schema are unchanged.

`uv run raceline doctor` includes a scalar-vs-array numerical smoke test for
this shim.
