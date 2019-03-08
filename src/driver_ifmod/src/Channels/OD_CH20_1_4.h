/* This file provides the definition of the struct AlgoIFOutputNoDI in the interface version ODA2_1.4.
   
   It also provides endianess-aware functions to safely convert a binary buffer to an 
   instance of this struct (if possible).
*/

#ifndef IFM_O3M_ALGOIFOUTPUTNODI_ODA2_1_4_CONVERTER_H_INCLUDED
#define IFM_O3M_ALGOIFOUTPUTNODI_ODA2_1_4_CONVERTER_H_INCLUDED

#include "../../include/ifm_types.h"

/* The struct has explicit padding, so that it should be usable on any target 
   without special compiler flags or pragmas related to padding.
*/
typedef struct
{
    /*
      Magic number of project, can be used to identify the output type.
    */
    ifm_o3m_sint8_t magic_no[4];
    /*
      Magic number of struct, can be used to identify the output type.
    */
    ifm_o3m_sint8_t struct_id[4];
    /*
      Version number of struct.
    */
    ifm_o3m_uint8_t version[2];
    ifm_o3m_uint8_t pad_001; /* explicit padding, do not access this member */
    ifm_o3m_uint8_t pad_002; /* explicit padding, do not access this member */
    /*
      Timestamp of the resulting data in masterclock domain. Copied from PMDRawData.
      
      Unit: [1e-6 s]
      Range: [0,MAX_UINT32]
    */
    ifm_o3m_uint32_t masterclockTimestamp;
    /*
      Rolling frame counter.
      
      Unit: [N/A]
      Range: [0,MAX_UINT32]
    */
    ifm_o3m_uint32_t frameCounter;
    /*
      availability flag         
              
      
      0 : available(available & 1) != 0: blockage detected(available & 2) != 0: attacker detected
      
      Unit: [bool]
      Range: [0,1]
    */
    ifm_o3m_uint32_t available;
    struct
    {
        /*
          X component of the camera's translation vector expressed in world coordinates.         
                  
          If the standard vehicle coordinate system is used, this value is the distance from rear axle to the
          camera's reference point parallel to the vehicle's driving direction. Positive values indicate that
          the camera is in front of the rear axle.
          
          Unit: [m]
          Range: [-30,30]
        */
        ifm_o3m_float32_t transX;
        /*
          Y component of the camera's translation vector expressed in world coordinates.         
                  
          If the standard vehicle coordinate system is used, this value is the distance from the vehicle's
          longitudinal center plane to the camera's reference point. Positive values indicate that the camera
          is located at the left side of the vehicle.
          
          Unit: [m]
          Range: [-30,30]
        */
        ifm_o3m_float32_t transY;
        /*
          Z component of the camera's translation vector expressed in world coordinates.         
                  
          If the standard vehicle coordinate system is used, this value is the distance from the ground plane
          to the camera's reference point. Positive values indicate that the camera is above the ground.
          
          Unit: [m]
          Range: [-30,30]
        */
        ifm_o3m_float32_t transZ;
        /*
          Camera rotation around the X axis.         
                  
          The camera orientation is expressed in so-called Euler angles.
          
          Unit: [rad]
          Range: [-pi,pi]
        */
        ifm_o3m_float32_t rotX;
        /*
          Camera rotation around the Y axis.         
                  
          The camera orientation is expressed in so-called Euler angles.
          
          Unit: [rad]
          Range: [-pi,pi]
        */
        ifm_o3m_float32_t rotY;
        /*
          Camera rotation around the Z axis.         
                  
          The camera orientation is expressed in so-called Euler angles.
          
          Unit: [rad]
          Range: [-pi,pi]
        */
        ifm_o3m_float32_t rotZ;
    } cameraCalibration;
    struct
    {
        /*
          Field of view, given as unit vectors in the world coordinate system.         
                  
          This is the vector (x,y,z) corresponding to the upper left corner.
        */
        ifm_o3m_float32_t upperLeft[3];
        /*
          Field of view, given as unit vectors in the world coordinate system.         
                  
          This is the vector (x,y,z) corresponding to the upper right corner.
        */
        ifm_o3m_float32_t upperRight[3];
        /*
          Field of view, given as unit vectors in the world coordinate system.         
                  
          This is the vector (x,y,z) corresponding to the lower left corner.
        */
        ifm_o3m_float32_t lowerLeft[3];
        /*
          Field of view, given as unit vectors in the world coordinate system.         
                  
          This is the vector (x,y,z) corresponding to the lower right corner.
        */
        ifm_o3m_float32_t lowerRight[3];
    } fieldOfView;
    struct
    {
        /*
          Focal length x direction.         
                  
          Negative in case of mirroring enabled for x direction. Zero in case of no 2D sensor present.
          
          Unit: [px]
          Range: [-MAX_FLOAT32,+MAX_FLOAT32]
        */
        ifm_o3m_float32_t intrCalib_2D_fx;
        /*
          Focal length y direction.         
                  
          Negative in case of mirroring enabled for y direction. Zero in case of no 2D sensor present.
          
          Unit: [px]
          Range: [-MAX_FLOAT32,+MAX_FLOAT32]
        */
        ifm_o3m_float32_t intrCalib_2D_fy;
        /*
          Main point in x.
          
          Unit: [px]
          Range: [-320,+320]
        */
        ifm_o3m_float32_t intrCalib_2D_mx;
        /*
          Main point in y.
          
          Unit: [px]
          Range: [-240,+240]
        */
        ifm_o3m_float32_t intrCalib_2D_my;
        /*
          skew parameter
          
          Unit: [N/A]
          Range: [-1,+1]
        */
        ifm_o3m_float32_t intrCalib_alpha;
        /*
          First radial distortion parameter.
          
          Unit: [N/A]
          Range: [-10,10]
        */
        ifm_o3m_float32_t intrCalib_k1;
        /*
          Second radial distortion parameter.
          
          Unit: [N/A]
          Range: [-10,10]
        */
        ifm_o3m_float32_t intrCalib_k2;
        /*
          Third radial distortion parameter.
          
          Unit: [N/A]
          Range: [-10,10]
        */
        ifm_o3m_float32_t intrCalib_k5;
        /*
          First tangential distortion parameter.
          
          Unit: [N/A]
          Range: [-10,10]
        */
        ifm_o3m_float32_t intrCalib_k3;
        /*
          Second tangential distortion parameter.
          
          Unit: [N/A]
          Range: [-10,10]
        */
        ifm_o3m_float32_t intrCalib_k4;
        /*
          x coordinate of center point for extrinsic calibration
          
          Unit: [N/A]
          Range: [-10,10]
        */
        ifm_o3m_float32_t extrCalib_center_tx;
        /*
          y coordinate of center point for extrinsic calibration
          
          Unit: [N/A]
          Range: [-10,10]
        */
        ifm_o3m_float32_t extrCalib_center_ty;
        /*
          z coordinate of center point for extrinsic calibration
          
          Unit: [N/A]
          Range: [-10,10]
        */
        ifm_o3m_float32_t extrCalib_center_tz;
        /*
          x coordinate of offset to ceneter point of extrinsic calibration
          
          Unit: [N/A]
          Range: [-10,10]
        */
        ifm_o3m_float32_t extrCalib_delta_tx;
        /*
          y coordinate of offset to ceneter point of extrinsic calibration
          
          Unit: [N/A]
          Range: [-10,10]
        */
        ifm_o3m_float32_t extrCalib_delta_ty;
        /*
          z coordinate of offset to ceneter point of extrinsic calibration
          
          Unit: [N/A]
          Range: [-10,10]
        */
        ifm_o3m_float32_t extrCalib_delta_tz;
        /*
          x rotation
          
          Unit: [rad]
          Range: [-pi,pi]
        */
        ifm_o3m_float32_t extrCalib_rot_x;
        /*
          y rotation
          
          Unit: [rad]
          Range: [-pi,pi]
        */
        ifm_o3m_float32_t extrCalib_rot_y;
        /*
          z rotation
          
          Unit: [rad]
          Range: [-pi,pi]
        */
        ifm_o3m_float32_t extrCalib_rot_z;
    } intrExtrCalib_2d;
    /*
      Absolute position of illumination in world coordinates [X,Y,Z].
      
      Unit: [m]
      Range: [-100,100]
    */
    ifm_o3m_float32_t illuPosition[3];
    /*
      Blockage ratio (for transmission over CAN)
      
      Unit: [N/A]
      Range: [0,1]
    */
    ifm_o3m_float32_t blockageRatio;
    /*
      Flag indicating if the blockage is available for this sample.
      
      Unit: [bool]
      Range: [0,1]
    */
    ifm_o3m_uint8_t blockageAvailable;
    ifm_o3m_uint8_t pad_003; /* explicit padding, do not access this member */
    ifm_o3m_uint8_t pad_004; /* explicit padding, do not access this member */
    ifm_o3m_uint8_t pad_005; /* explicit padding, do not access this member */
    struct
    {
        struct
        {
            /*
              The ID of the object.         
                      
              An id of 0 indicates invalid objects.
              
              Unit: [N/A]
              Range: [0,255]
            */
            ifm_o3m_sint32_t id;
            /*
              A toggle bit which can be used to detect whether an object is new on the bus or not.         
                      
              Note: This is necessary because the sensor has an internal list of objects and from these objects a
              maximum number of 20 objects are sent over the external interfaces. During the lifetime of an
              object (identified by its id) this bit does not change. If a new track is assigned to the id, this
              bit toggles once when the object is sent out for the first time.
              
              Unit: [N/A]
              Range: [0,1]
            */
            ifm_o3m_sint32_t history;
            /*
              A bit indicating whether this object has been measured in this frame.         
                      
              
              0: Object has been predicted
              1: Object has been measured
              
              Unit: [N/A]
              Range: [0,1]
            */
            ifm_o3m_sint32_t measured;
            /*
              The type of the object.         
                      
              
              0: Normal object (tracked with the "microtrack" approach)
              1: Retroreflector object (tracked by its center of gravity)
              
              Unit: [N/A]
              Range: [0,1]
            */
            ifm_o3m_sint32_t type;
            /*
              Age of the object in frames.
              
              Unit: [frames]
              Range: [0,INT32_MAX]
            */
            ifm_o3m_sint32_t age;
            /*
              x coordinate of object's first point.         
                      
              Note: For retroreflector objects, this is the minimum x coordinate of the retroreflector. Otherwise
              this is the x component of the "flat" model's first point.
              
              Unit: [m]
              Range: [-200,+200]
            */
            ifm_o3m_float32_t x1;
            /*
              y coordinate of object's first point.         
                      
              Note: For retroreflector objects, this is the minimum y coordinate of the retroreflector. Otherwise
              this is the y component of the "flat" model's first point.
              
              Unit: [m]
              Range: [-200,+200]
            */
            ifm_o3m_float32_t y1;
            /*
              x coordinate of object's second point.         
                      
              Note: For retroreflector objects, this is the maximum x coordinate of the retroreflector. Otherwise
              this is the x component of the "flat" model's second point.
              
              Unit: [m]
              Range: [-200,+200]
            */
            ifm_o3m_float32_t x2;
            /*
              y coordinate of object's second point.         
                      
              Note: For retroreflector objects, this is the maximum y coordinate of the retroreflector. Otherwise
              this is the y component of the "flat" model's second point.
              
              Unit: [m]
              Range: [-200,+200]
            */
            ifm_o3m_float32_t y2;
            /*
              minimum z coordinate of object.
              
              Unit: [m]
              Range: [-200,+200]
            */
            ifm_o3m_float32_t zMin;
            /*
              maximum z coordinate of object.
              
              Unit: [m]
              Range: [-200,+200]
            */
            ifm_o3m_float32_t zMax;
            /*
              X component of the object's relative velocity.
              
              Unit: [m/s]
              Range: [-200,+200]
            */
            ifm_o3m_float32_t vX;
            /*
              Y component of the object's relative velocity.
              
              Unit: [m/s]
              Range: [-200,+200]
            */
            ifm_o3m_float32_t vY;
            /*
              Z component of the object's relative velocity.         
                      
              Note: This information is only valid for retroreflector objects Otherwise this value is set to 0.
              
              Unit: [m/s]
              Range: [-200,+200]
            */
            ifm_o3m_float32_t vZ;
            /*
              X component of the object's relative acceleration.
              
              Unit: [m/s^2]
              Range: [-50,+50]
            */
            ifm_o3m_float32_t aX;
            /*
              Y component of the object's relative acceleration.
              
              Unit: [m/s^2]
              Range: [-50,+50]
            */
            ifm_o3m_float32_t aY;
            /*
              Z component of the object's relative acceleration.         
                      
              Note: This information is only valid for retroreflector objects. Otherwise this value is set to 0.
              
              Unit: [m/s^2]
              Range: [-50,+50]
            */
            ifm_o3m_float32_t aZ;
            /*
              Measure for the existence probability of an object.         
                      
              This is a value between 0 and 1, where 0 indicates a low probability that an object exists at the
              given position and 1 indicates a high probability that an object exists at the given position.
              
              Unit: [probability]
              Range: [0,1]
            */
            ifm_o3m_float32_t existenceProbability;
            /*
              Quality of the X-velocity.         
                      
              This is a value between 0 and 1, where 0 indicates unreliable information in the x component of the
              velocity and 1 indicates reliable information in the x component of the velocity.Note: This
              information is not available for retroreflector objects. In this mode, the velocity quality values
              are set to 1.
              
              Unit: [N/A]
              Range: [0,1]
            */
            ifm_o3m_float32_t vXQuality;
            /*
              Quality of the Y-velocity.         
                      
              This is a value between 0 and 1, where 0 indicates unreliable information in the y component of the
              velocity and 1 indicates reliable information in the y component of the velocity.Note: This
              information is not available for retroreflector objects. In this mode, the velocity quality values
              are set to 1.
              
              Unit: [N/A]
              Range: [0,1]
            */
            ifm_o3m_float32_t vYQuality;
            /*
              Distance of the object to the ego vehicle.         
                      
              For normal non-retroreflector objects, this is the minimal 2D distance from the ego vehicle
              outlines to the object outline.For retroreflector objects, this value is set to 0.0.
              
              Unit: [m]
              Range: [0,200]
            */
            ifm_o3m_float32_t distanceToEgo;
        } objectList[20];
        struct
        {
            /*
              crash prediction flag         
                      
              
              0: no crash predicted
              1: crash predicted
              -1: in down time after loss of last crash
              -2: crash predictor deactivated
              
              Note: This signal is filtered over time to avoid unnecessary toggling.
              
              Unit: [N/A]
              Range: [-2,1]
            */
            ifm_o3m_sint32_t crashPredicted;
            /*
              time to collision (TTC) for predicted crash (-1 if no crash predicted).
              
              Unit: [s]
              Range: [-1, inf]
            */
            ifm_o3m_float32_t crashPredictTime;
            /*
              impact velocity for predicted crash (-1 if no crash predicted).
              
              Unit: [m/s]
              Range: [-1, 50]
            */
            ifm_o3m_float32_t relImpactVelocity;
            /*
              ID of the crash object (0 if no crash predicted).         
                      
              Note: Because of the filtering of the crashPredicted signal, this signal might be 0 even though
              crashPredicted is 1.
              
              Unit: [N/A]
              Range: [0, 255]
            */
            ifm_o3m_sint32_t crashObjectID;
            /*
              criticality for predicted crash (0 if no crash predicred)         
                      
              
              0: no crash relevant object available.1: most critical (based on shortest configured reaction
              time)2: medium critical (based on medium configured reaction time)3: least critical (based on
              largest configured reaction time)
              
              Unit: [N/A]
              Range: [0, 3]
            */
            ifm_o3m_sint32_t criticality;
            /*
              flag indicating whether the predicted collision is based on the minimum zone         
                      
              
              0: predicted collision is based on dynamic paths1: predicted collision is based on minimum zone
              
              Unit: [N/A]
              Range: [0,1]
            */
            ifm_o3m_sint32_t minimumZoneTriggered;
            struct
            {
                /*
                  state of the current zone         
                          
                  
                  -1: disabled (invalid zone)0: not occupied1: occupied
                  Note: This signal is filtered over time to avoid unnecessary toggling.
                  
                  Unit: [N/A]
                  Range: [-1,1]
                */
                ifm_o3m_sint32_t zoneState;
                /*
                  ID of the nearest Object occupying the zone (0 if no such object is available).         
                          
                  Note: Because of the filtering of zoneState, this signal might be 0 even though zoneState is 1.
                  
                  Unit: [N/A]
                  Range: [0,255]
                */
                ifm_o3m_sint32_t objectID;
            } zoneResults[3];
        } crashPredictorResult;
    } objectDetectionResult;
    struct
    {
        struct
        {
            /*
              valid flag for calibration
            */
            ifm_o3m_sint32_t calibValid;
            /*
              Stable calibration counter, can be used as an acceptance criterium.
            */
            ifm_o3m_sint32_t calibrationStableCounter;
            struct
            {
                /*
                  X component of the camera's translation vector expressed in world coordinates.         
                          
                  If the standard vehicle coordinate system is used, this value is the distance from rear axle to the
                  camera's reference point parallel to the vehicle's driving direction. Positive values indicate that
                  the camera is in front of the rear axle.
                  
                  Unit: [m]
                  Range: [-30,30]
                */
                ifm_o3m_float32_t transX;
                /*
                  Y component of the camera's translation vector expressed in world coordinates.         
                          
                  If the standard vehicle coordinate system is used, this value is the distance from the vehicle's
                  longitudinal center plane to the camera's reference point. Positive values indicate that the camera
                  is located at the left side of the vehicle.
                  
                  Unit: [m]
                  Range: [-30,30]
                */
                ifm_o3m_float32_t transY;
                /*
                  Z component of the camera's translation vector expressed in world coordinates.         
                          
                  If the standard vehicle coordinate system is used, this value is the distance from the ground plane
                  to the camera's reference point. Positive values indicate that the camera is above the ground.
                  
                  Unit: [m]
                  Range: [-30,30]
                */
                ifm_o3m_float32_t transZ;
                /*
                  Camera rotation around the X axis.         
                          
                  The camera orientation is expressed in so-called Euler angles.
                  
                  Unit: [rad]
                  Range: [-pi,pi]
                */
                ifm_o3m_float32_t rotX;
                /*
                  Camera rotation around the Y axis.         
                          
                  The camera orientation is expressed in so-called Euler angles.
                  
                  Unit: [rad]
                  Range: [-pi,pi]
                */
                ifm_o3m_float32_t rotY;
                /*
                  Camera rotation around the Z axis.         
                          
                  The camera orientation is expressed in so-called Euler angles.
                  
                  Unit: [rad]
                  Range: [-pi,pi]
                */
                ifm_o3m_float32_t rotZ;
            } calibResult;
        } commonCalibrationResult;
        struct
        {
            /*
              Number of triangles detected.
              
              Unit: [N/A]
              Range: [0,8]
            */
            ifm_o3m_uint8_t numTrianglesDetected;
            ifm_o3m_uint8_t pad_001; /* explicit padding, do not access this member */
            ifm_o3m_uint8_t pad_002; /* explicit padding, do not access this member */
            ifm_o3m_uint8_t pad_003; /* explicit padding, do not access this member */
            struct
            {
                /*
                  score of triangle detection (-1: inverse match, 1: perfect match)
                  
                  Unit: [N/A]
                  Range: [-1,1]
                */
                ifm_o3m_float32_t score;
                /*
                  Cartesian 3D position of triangle center (x, y, z)
                */
                ifm_o3m_float32_t pos3D[3];
                /*
                  Corner 2D positions of triangle corners (for image overlay).         
                          
                  The first index represents the corner point index in the triangle. The second index represents the
                  x (=0) or y (=1) coordinate on the 2D image plane.
                */
                ifm_o3m_float32_t corners[3][2];
            } triangleDetections[8];
            /*
              Boolean indicating a valid Pattern Autocalibration frame.
            */
            ifm_o3m_sint32_t frameValid;
            /*
              Mean calibration reprojection error per frame.
              
              Unit: [m]
              Range: [0, 0.25]
            */
            ifm_o3m_float32_t frameReprojectError;
        } pacCalibrationResult;
        struct
        {
            /*
              Set to 1 if street estimation is considered valid and may be used for further processing.
            */
            ifm_o3m_sint32_t planeValid;
            struct
            {
                /*
                  Pitch angle of the camera in [rad].
                */
                ifm_o3m_float32_t pitchAngle;
                /*
                  Roll angle of the camera in [rad].
                */
                ifm_o3m_float32_t rollAngle;
                /*
                  height of the camera in [m].
                */
                ifm_o3m_float32_t camHeight;
                /*
                  normal vector of the currently estimated road plane (x component)         
                          
                  This is derived directly from pitchAngle and rollAngle and is provided for convenience.
                */
                ifm_o3m_float32_t normalx;
                /*
                  normal vector of the currently estimated road plane (y component)         
                          
                  This is derived directly from pitchAngle and rollAngle and is provided for convenience.
                */
                ifm_o3m_float32_t normaly;
                /*
                  normal vector of the currently estimated road plane (z component)         
                          
                  This is derived directly from pitchAngle and rollAngle and is provided for convenience.
                */
                ifm_o3m_float32_t normalz;
            } planeEstimation;
            /*
              Plausibility [0,1] for the estimated plane parameters.
            */
            ifm_o3m_float32_t plausibility;
            /*
              Standard deviation of distance to plane of all pixels, which are assigned as street pixels.
            */
            ifm_o3m_float32_t distanceDeviation;
        } streetCalibrationResult;
    } calibrationResult;
    struct
    {
        /*
          the digitial outputs of the logic graph         
                  
          This output should be assigned with PVO nodes addressing ChannelID 2, expression
          "AlgoState.algoLogicState.ifOutput.digitalOutput[]". Using this pattern, it is possible to treat
          the digital outputs as a kind of external memory for the logic graph.
        */
        ifm_o3m_uint8_t digitalOutput[100];
        /*
          the analog outputs of the logic graph         
                  
          This output should be assigned with PVO nodes addressing ChannelID 2, expression
          "AlgoState.algoLogicState.ifOutput.analogOuptut[]". Using this pattern, it is possible to treat the
          analog outputs as a kind of external memory for the logic graph.
        */
        ifm_o3m_float32_t analogOutput[20];
    } logicOutput;
} ifm_o3m_AlgoIFOutputNoDI_ODA2_1_4;

        
/* Casts the buffer to ifm_o3m_AlgoIFOutputNoDI_ODA2_1_4 (if possible) and returns a pointer to it.
   Use this function on little Endian systems.

   Returns NULL in case of errors. */
ifm_o3m_AlgoIFOutputNoDI_ODA2_1_4* ifm_o3m_ConvertBufferToLittleEndian_ODA2_1_4(void *buffer, ifm_o3m_uint32_t bufferSize);

/* Converts the endianess of the buffer to native form and returns a pointer to ifm_o3m_AlgoIFOutputNoDI_ODA2_1_4. 
   Note: The original buffer is modified in place. 
   Use this function on big Endian systems.
   
   Returns NULL in case of errors. */
ifm_o3m_AlgoIFOutputNoDI_ODA2_1_4* ifm_o3m_ConvertBufferToBigEndian_ODA2_1_4(void *buffer, ifm_o3m_uint32_t bufferSize);

#endif
