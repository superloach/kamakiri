# physac-go
A port of Raylib's [Physac](https://github.com/victorfisac/Physac) in pure Go.

## notable changes
 - Excessive use of the words `Physics` or `Data` removed.
 - `Vector2` is now `XY` for simplicity.
 - Global vars/defines have been moved to a struct called `World`.
 - Slices are used instead of arrays.
 - `float64` used for both `float` and `double`, for Go's `math` lib.
 - Changed `Create` to `New` per Go conventions.
 - Don't automatically spawn a thread/goroutine - use `RunStep`.
 - Remove getters/setters when possible.
 - Use slices of vertices (`[]Vertex`) instead of `Polygon`s
 - Move `Init` functionality into `New` functions.
 - Rename `Manifold` to `Contact`.
 - Make the `Math*` and `Mat2*` functions methods on respective types.
