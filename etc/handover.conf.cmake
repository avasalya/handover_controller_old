{
  // Log bin files in LogDirectory
  "Log": true,

  // LogDirectory dictates where the log files will be stored, defaults to /tmp
  "LogDirectory" : "/tmp",

  // Log files will have the name [LogTemplate]-[ControllerName]-[date].log
  "LogTemplate" : "mc-control",

  // When true, the FSM transitions are managed by an external tool
  "Managed": false,

  // When true and the FSM is self-managed, transitions should be triggered
  "StepByStep": false,

  // Where to look for state libraries
//  "StatesLibraries": ["@MC_RTC_LIBDIR@/mc_controller/mc_torquing_controller/states", "@MC_RTC_LIBDIR@/mc_controller/fsm/states"],

  // Where to look for state files
  // "StatesFiles": ["@MC_RTC_LIBDIR@/mc_controller/mc_torquing_controller/states/data"],
  "StatesFiles": [],

  // If true, state factory will be more verbose
  "VerboseStateFactory": true,

  // Controller is created in a sandbox, which in between a thread and a fork;
  // try to keep it to false, as it can create weird conflicts with threads
  "UseSandbox" : false,

  // If we are in simulation, we don't run the Admittance and we use simulated markers
  "Simulation" : false,

  // // Measured value from the real breaker
  // "identifyForceSensor": true,
  // "gravityForce": 19.052,
  // "facom_CoM": [0.0639648, -0.00898009, -0.0781332],
  // "wrench_offset": [-0.0539328, 0.312478, -0.18967, 3.26464, 1.91082, 5.08879],
  // "gravityForce_supportHand": 10.3571,
  // "facom_CoM_supportHand": [0.003,0.0013,-0.034],
  // "wrench_offset_supportHand": [10.0,20.0,30.0,10.0,20.0,30.0],
  // "gainsMX": [800.0,600.0,13.0],
  // "gainsMY": [800.0,600.0,13.0],
  // "gainsMZ": [800.0,600.0,13.0],
  // "gainsFX": [2000.0,1600.0,20.0],
  // "gainsFY": [2000.0,1600.0,20.0],
  // "gainsFZ": [2000.0,1600.0,20.0],
  // "gainM2Phi": 0.009,
  // "gainF2X": 0.003,
  // "offsetVisServo_toolGrab": [-0.32,0.0,0.0],
  // "offsetVisServo_rail": [0.0,-0.05,0.0],
  // "offsetVisServo_0": [-0.125,-0.076,0.0],
  // "offsetVisServo_1": [-0.025,-0.078,0.0],
  // "wingLengthLShapeRail": 0.08,
  // "wingLengthLShapeTool": 0.04,
  // "wingLengthLShapeWall_0": 0.05,
  // "wingLengthLShapeWall_1": 0.06,
  // "wingLengthLShapeTolerance": 0.005,

  // Additional robots to load
  "robots":
  {
    "ground":
    {
      "module": "env",
      "params": ["@AROBASE@MC_ENV_DESCRIPTION@AROBASE@", "ground"]
    },
  },

  //
  // Task configuration
  //

  "constraints":
  [
    {
      "type": "contact"
    },
    {
      "type": "dynamics",
      "robotIndex": 0,
      "damper": [0.1, 0.01, 0.5]
    }
  ],
  "collisions":
  [
    {
      "type": "collision",
      "r1Index": 0,
      "r2Index": 0,
      "useMinimal": true
    }
  ],
  "contacts":
  [
    {
      "r1": "hrp2_drc",
      "r2": "ground",
      "r1Surface": "LFullSole",
      "r2Surface": "AllGround"
    },
    {
      "r1": "hrp2_drc",
      "r2": "ground",
      "r1Surface": "RFullSole",
      "r2Surface": "AllGround"
    }
  ],
  "hrp2":
  {
    "posture":
    {
      "stiffness": 1.0,
      "weight": 10.0
    }
  },

  //
  // FSM configuration
  //
  
  // "configs":
  // {
  //   "ApproachToolStep":
  //   {
  //     "threshold_eval": 0.05,
  //     "threshold_speed": 0.001,
  //     "stiffness": 3.0,
  //     "weight": 5000.0,
  //     "toolApproachWaypoint1" : [-0.2,0.5,0.1],
  //     "toolApproachWaypoint2" : [-0.2,0.5,0.2],
  //     "toolApproachWaypoint3" : [-0.2,0.5,0.2]
  //   },
  //   "ContactToolStep":
  //   {
  //     "threshold_eval": 0.05,
  //     "threshold_speed": 0.001,
  //     "stiffness": 3.0,
  //     "weight": 5000.0,
  //     "toolContactWaypoint1" : [0.0,-0.05,0.0],
  //     "nrPointsTraj" : 1000
  //   },
  //   "GraspToolStep":
  //   {
  //     "gripperVal": -0.45
  //   },
  //   "RemoveToolStep":
  //   {
  //     "threshold_eval": 0.01,
  //     "threshold_speed": 0.001,
  //     "stiffness": 3.0,
  //     "weight": 5000.0,
  //     "nrPointsTraj": 1000,
  //     "toolRemoveWaypoint1": [0.0,0.2,0.0],
  //     "toolRemoveWaypoint2" : [0.0,0.0,0.0]
  //   },
  //   "GoHalfSittingStep":
  //   {
  //     "threshold_eval": 0.01,
  //     "threshold_speed": 0.001,
  //     "stiffness": 3.0,
  //     "weight": 5000.0,
  //     "nrPointsTraj": 1000,
  //     "toolRemoveWaypoint1": [0.15,0.175,-0.2]
  //   },
  //   "SensorIdentificationStep":
  //   {
  //     "threshold_eval": 0.01,
  //     "threshold_speed": 0.005,
  //     "stiffness": 3.0,
  //     "weight": 5000.0,
  //     "nrPointsTraj": 4500
  //   },
  //   "ApproachWallStep":
  //   {
  //     "threshold_eval": 0.03,
  //     "threshold_speed": 0.001,
  //     "stiffness": 3.0,
  //     "weight": 5000.0,
  //     "nrPointsTraj": 1250
  //   },
  //   "IdentifyNonIdentifiedForceOffset":
  //   {
  //     "nrPointsTraj": 500
  //   },
  //   "visServoToBoltStep":
  //   {
  //     "threshold_eval": 0.01,
  //     "threshold_speed": 0.005,
  //     "stiffness": 3.0,
  //     "weight": 5000.0,
  //     "nrPointsTraj": 500
  //   },
  //   "ContactWallVisServoStep":
  //   {
  //     "threshold_eval": 0.01,
  //     "threshold_speed": 0.005,
  //     "stiffness": 3.0,
  //     "weight": 5000.0,
  //     "forceThreshold": 10.0
  //   },
  //   "InsertContactTurnStep":
  //   {
  //     "threshold_eval": 0.01,
  //     "threshold_speed": 0.005,
  //     "stiffness": 3.0,
  //     "weight": 5000.0,
  //     "nrPointsTraj": 1000
  //   },
  //   "RelaxAfterInsertionStep":
  //   {
  //     "threshold_eval": 0.01,
  //     "threshold_speed": 0.005,
  //     "stiffness": 3.0,
  //     "weight": 5000.0,
  //     "nrPointsTraj": 1000
  //   },
  //   "CheckConstactStep":
  //   {
  //     "threshold_eval": 0.01,
  //     "threshold_speed": 0.005,
  //     "stiffness": 3.0,
  //     "weight": 5000.0,
  //     "nrPointsTraj": 1000
  //   },
  //   "TighteningStep":
  //   {
  //     "threshold_eval": 0.01,
  //     "threshold_speed": 0.005,
  //     "stiffness": 3.0,
  //     "weight": 5000.0,
  //     "nmbrTightScrew": 30,
  //     "nrPointsTraj_0": 1800,
  //     "nrPointsTraj_1": 1000,
  //     // angle in degree (conversion to radians is done in step)
  //     "toolAngle_0": 70,
  //     "toolAngle_1": 45
  //   },
  //   "RelaxAfterTighteningStep":
  //   {
  //     "threshold_eval": 0.01,
  //     "threshold_speed": 0.005,
  //     "stiffness": 3.0,
  //     "weight": 5000.0,
  //     "nrPointsTraj": 1500
  //   },
  //   "RemoveContactStep":
  //   {
  //     "threshold_eval": 0.01,
  //     "threshold_speed": 0.005,
  //     "stiffness": 3.0,
  //     "weight": 5000.0,
  //     "nrPointsTraj": 2000
  //   },
  //   "GoToOriginalPositionStep":
  //   {
  //     "threshold_eval": 0.01,
  //     "threshold_speed": 0.005,
  //     "stiffness": 3.0,
  //     "weight": 5000.0,
  //     "nrPointsTraj": 500
  //   }
  // },
  // "init": "Start",
  // "states":
  // {
  //   "Start":
  //   {
  //     "base": "Pause"
  //   }
  // },
  // "transitions":
  // [
  //   // Grab the tool then identification
  //   // ["Start", "OK", "ApproachToolStep", "Strict"],
  //   // ["ApproachToolStep", "OK", "ContactToolStep", "Strict"],
  //   // ["ContactToolStep", "OK", "GraspToolStep", "Strict"],
  //   // ["GraspToolStep", "OK", "RemoveToolStep", "Strict"],
  //   // ["RemoveToolStep", "OK", "GoHalfSittingStep", "Strict"],
  //   // ["GoHalfSittingStep", "SkipIdentification", "ApproachWallStep", "Strict"],
  //   // ["GoHalfSittingStep", "OK", "SensorIdentificationStep", "Strict"],
  //   // ["SensorIdentificationStep", "OK", "ApproachWallStep", "Strict"],
  //   // Tool in hand then identification
  //   ["Start", "OK", "SensorIdentificationStep", "Strict"],
  //   ["SensorIdentificationStep", "OK", "ApproachWallStep", "Strict"],
  //   // Tool in hand, no identification
  //   //["Start", "OK", "ApproachWallStep", "Strict"],
  //   ["ApproachWallStep", "OK", "IdentifyNonIdentifiedForceOffset", "Strict"],
  //   ["IdentifyNonIdentifiedForceOffset", "OK", "visServoToBoltStep", "Strict"],
  //   ["visServoToBoltStep", "OK", "ContactWallVisServoStep",  "Strict"],
  //   ["ContactWallVisServoStep", "MissedTarget", "GoToOriginalPositionStep", "Strict"],
  //   ["ContactWallVisServoStep", "OK", "InsertContactTurnStep", "Strict"],
  //   ["InsertContactTurnStep", "Repeat", "InsertContactTurnStep", "Auto"],
  //   ["InsertContactTurnStep", "OK", "RelaxAfterInsertionStep", "Strict"],
  //   ["RelaxAfterInsertionStep", "NoTightening", "RelaxAfterTighteningStep", "Strict"],
  //   ["RelaxAfterInsertionStep", "OK", "TighteningStep", "Strict"],
  //   // ["RelaxAfterInsertionStep", "OK", "CheckContactStep", "Strict"],
  //   // ["CheckContactStep", "NotInserted", "RelaxAfterTighteningStep", "Strict"],
  //   // ["CheckContactStep", "OK", "TighteningStep", "Strict"],
  //   ["TighteningStep", "Repeat", "TighteningStep", "Auto"],
  //   ["TighteningStep", "OK", "RelaxAfterTighteningStep", "Strict"],
  //   ["RelaxAfterTighteningStep", "OK", "RemoveContactStep", "Strict"],
  //   ["RemoveContactStep", "Repeat", "RelaxAfterTighteningStep", "Strict"],
  //   ["RemoveContactStep", "OK", "GoToOriginalPositionStep", "Strict"],
  //   ["GoToOriginalPositionStep", "TryAgain", "IdentifyNonIdentifiedForceOffset", "Strict"],
  //   ["GoToOriginalPositionStep", "OK", "ApproachWallStep", "Strict"]
  // ],
}
