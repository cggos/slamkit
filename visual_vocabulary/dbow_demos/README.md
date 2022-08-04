# DBoW Demos

---

# Build

```sh
mkdir build
cd build
cmake .. [-DWITH_DBOW2=0 -DWITH_DBOW3=1] # set one on
make -j4
```

# Run

* train Voc
  ```sh
  ./create_voc <data-dir>
  ```

* use Voc
  ```sh
  ./loop_closure <data-dir>
  ```