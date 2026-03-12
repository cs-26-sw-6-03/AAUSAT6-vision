```
aausat6-vision/
├── CMakeLists.txt
├── README.md
├── .gitignore
│
├── config/
│   ├── default.yaml                  # Default parameters
│   └── experiments/
│       ├── baseline.yaml
│       ├── orb_only.yaml
│       └── optical_flow_only.yaml
│
├── src/
│   ├── main.cpp
│   │
│   ├── cli/
│   │   ├── CLI.hpp/.cpp              # Argument parsing + dispatch
│   │   └── Commands.hpp/.cpp         # run, db, inspect, experiment
│   │
│   ├── pipeline/
│   │   ├── Pipeline.hpp/.cpp         # Wires stages, owns router logic
│   │   ├── Stage.hpp                 # Abstract base
│   │   ├── ThreadedStage.hpp/.cpp    # Stage + thread + queue
│   │   ├── Router.hpp/.cpp           # Reads FrameContext flags, dispatches
│   │   └── FrameContext.hpp          # Shared state passed between stages
│   │
│   ├── stages/
│   │   ├── CaptureStage.hpp/.cpp     # GStreamer input
│   │   ├── OpticalFlowStage.hpp/.cpp # Lucas-Kanade, sets tracking_score
│   │   ├── OrbStage.hpp/.cpp         # ORB detect + describe
│   │   ├── MatchingStage.hpp/.cpp    # Hamming + ratio test
│   │   ├── RansacStage.hpp/.cpp      # EG-RANSAC, inlier filtering
│   │   ├── PoseStage.hpp/.cpp        # Pose estimation from inliers
│   │   └── OutputStage.hpp/.cpp      # GStreamer output stream
│   │
│   ├── db/
│   │   ├── KeypointDB.hpp/.cpp       # CRUD interface
│   │   ├── DBSchema.hpp              # KeypointEntry, DescriptorRecord
│   │   └── DBSerializer.hpp/.cpp     # SQLite backend
│   │
│   └── utils/
│       ├── Logger.hpp/.cpp
│       ├── Config.hpp/.cpp           # YAML loader, typed accessors
│       └── ThreadSafeQueue.hpp       # Template queue used by all stages
│
├── data/
│   └── keypoints.db
│
├── tests/
│   ├── CMakeLists.txt
│   ├── test_pipeline.cpp
│   ├── test_optical_flow.cpp
│   ├── test_orb.cpp
│   ├── test_matching.cpp
│   ├── test_ransac.cpp
│   ├── test_db.cpp
│   └── test_cli.cpp
│
├── scripts/
│   ├── build.sh
│   └── run_experiment.sh
│
└── docs/
    ├── pipeline.md                   # Pipeline architecture
    └── db_schema.md
```