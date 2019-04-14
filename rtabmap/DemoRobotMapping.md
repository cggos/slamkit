# Demo Robot Mapping

-----

[TOC]

## Introduction

* Command

```
roslaunch rtabmap_ros demo_robot_mapping.launch rtabmapviz:=false rviz:=true
```

* Node Graph

![demo_robot_mapping_graph](./images/rtabmap_demo_robot_mapping_graph.png)

## Nodes

### /rtabmap/rtabmap

* 流程图

```dot
digraph g {
    rankdir = LR;

    subgraph cluster0{
        label="Parameters";
        parameters_;
    }

    subgraph cluster1{
        label="MapsManager";
        initMaps;
    }

    subgraph cluster2{
        label="CoreWrapper Nodelet";
        onInit;
        Callback;
    }

    subgraph cluster3{
        label="RTABMap Corelib";
        initRTABMap;
        process;
    }

    subgraph cluster4{
        label="Memory";
        newMemory;
        initMemory;
        loadDataFromDb;
    }

    subgraph cluster5{
        label="DBDriver";
        createDB;
    }
    subgraph cluster6{
        node [shape=box, size="20,20"];
        {
              Feature2D_create;
              newVWDictionary;
              Registration_create;
              newRegistrationIcp;
              newOccupancyGrid;
        }
    }

    CoreNode -> onInit [label="nodelet::load()"];

    onInit -> parameters_;
    onInit -> initMaps;
    onInit -> initRTABMap;
    onInit -> Callback;

    Callback -> process;

    initRTABMap -> newMemory;
    initRTABMap -> initMemory;

    newMemory -> Feature2D_create [lhead=cluster6];

    initMemory -> createDB;
    initMemory -> loadDataFromDb;
}
```

### /points_xyzrgb
* 流程图
```dot
digraph g{
    rankdir = LR;
    rviz -> points_xyzrgb [label="launch nodelet"];
}
```
