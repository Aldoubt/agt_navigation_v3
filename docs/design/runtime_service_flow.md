# Runtime Service Flow

## ApplyMapRuntime

Request:

- map_id
- version
- generation

Flow:

```
request
  |
  v
validate MapPackage
  |
  v
check generation consistency
  |
  v
prepare Nav2 backend
  |
  v
prepare Localization backend
  |
  v
activate backends
  |
  v
publish READY
```

Failure at any stage publishes ERROR.

## Design Rule

Runtime Bridge coordinates state. It does not implement SLAM, localization or planning algorithms.
