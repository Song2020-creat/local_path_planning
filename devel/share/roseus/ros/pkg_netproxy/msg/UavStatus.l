;; Auto-generated. Do not edit!


(when (boundp 'pkg_netproxy::UavStatus)
  (if (not (find-package "PKG_NETPROXY"))
    (make-package "PKG_NETPROXY"))
  (shadow 'UavStatus (find-package "PKG_NETPROXY")))
(unless (find-package "PKG_NETPROXY::UAVSTATUS")
  (make-package "PKG_NETPROXY::UAVSTATUS"))

(in-package "ROS")
;;//! \htmlinclude UavStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pkg_netproxy::UavStatus
  :super ros::object
  :slots (_header _roll _pitch _yaw _latitude _longitude _altitude _baro_altitude _tf_altitude _velocity_n _velocity_e _velocity_d _g_latitude _g_longitude _g_altitude ))

(defmethod pkg_netproxy::UavStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:roll __roll) 0.0)
    ((:pitch __pitch) 0.0)
    ((:yaw __yaw) 0.0)
    ((:latitude __latitude) 0.0)
    ((:longitude __longitude) 0.0)
    ((:altitude __altitude) 0.0)
    ((:baro_altitude __baro_altitude) 0.0)
    ((:tf_altitude __tf_altitude) 0.0)
    ((:velocity_n __velocity_n) 0.0)
    ((:velocity_e __velocity_e) 0.0)
    ((:velocity_d __velocity_d) 0.0)
    ((:g_latitude __g_latitude) 0.0)
    ((:g_longitude __g_longitude) 0.0)
    ((:g_altitude __g_altitude) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _roll (float __roll))
   (setq _pitch (float __pitch))
   (setq _yaw (float __yaw))
   (setq _latitude (float __latitude))
   (setq _longitude (float __longitude))
   (setq _altitude (float __altitude))
   (setq _baro_altitude (float __baro_altitude))
   (setq _tf_altitude (float __tf_altitude))
   (setq _velocity_n (float __velocity_n))
   (setq _velocity_e (float __velocity_e))
   (setq _velocity_d (float __velocity_d))
   (setq _g_latitude (float __g_latitude))
   (setq _g_longitude (float __g_longitude))
   (setq _g_altitude (float __g_altitude))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:altitude
   (&optional __altitude)
   (if __altitude (setq _altitude __altitude)) _altitude)
  (:baro_altitude
   (&optional __baro_altitude)
   (if __baro_altitude (setq _baro_altitude __baro_altitude)) _baro_altitude)
  (:tf_altitude
   (&optional __tf_altitude)
   (if __tf_altitude (setq _tf_altitude __tf_altitude)) _tf_altitude)
  (:velocity_n
   (&optional __velocity_n)
   (if __velocity_n (setq _velocity_n __velocity_n)) _velocity_n)
  (:velocity_e
   (&optional __velocity_e)
   (if __velocity_e (setq _velocity_e __velocity_e)) _velocity_e)
  (:velocity_d
   (&optional __velocity_d)
   (if __velocity_d (setq _velocity_d __velocity_d)) _velocity_d)
  (:g_latitude
   (&optional __g_latitude)
   (if __g_latitude (setq _g_latitude __g_latitude)) _g_latitude)
  (:g_longitude
   (&optional __g_longitude)
   (if __g_longitude (setq _g_longitude __g_longitude)) _g_longitude)
  (:g_altitude
   (&optional __g_altitude)
   (if __g_altitude (setq _g_altitude __g_altitude)) _g_altitude)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _roll
    4
    ;; float32 _pitch
    4
    ;; float32 _yaw
    4
    ;; float64 _latitude
    8
    ;; float64 _longitude
    8
    ;; float64 _altitude
    8
    ;; float32 _baro_altitude
    4
    ;; float32 _tf_altitude
    4
    ;; float32 _velocity_n
    4
    ;; float32 _velocity_e
    4
    ;; float32 _velocity_d
    4
    ;; float64 _g_latitude
    8
    ;; float64 _g_longitude
    8
    ;; float64 _g_altitude
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _longitude
       (sys::poke _longitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _altitude
       (sys::poke _altitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _baro_altitude
       (sys::poke _baro_altitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tf_altitude
       (sys::poke _tf_altitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _velocity_n
       (sys::poke _velocity_n (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _velocity_e
       (sys::poke _velocity_e (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _velocity_d
       (sys::poke _velocity_d (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _g_latitude
       (sys::poke _g_latitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _g_longitude
       (sys::poke _g_longitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _g_altitude
       (sys::poke _g_altitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _roll
     (setq _roll (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch
     (setq _pitch (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _latitude
     (setq _latitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _longitude
     (setq _longitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _altitude
     (setq _altitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _baro_altitude
     (setq _baro_altitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tf_altitude
     (setq _tf_altitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _velocity_n
     (setq _velocity_n (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _velocity_e
     (setq _velocity_e (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _velocity_d
     (setq _velocity_d (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _g_latitude
     (setq _g_latitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _g_longitude
     (setq _g_longitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _g_altitude
     (setq _g_altitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get pkg_netproxy::UavStatus :md5sum-) "1e2778f4d8e4263c2ff410b965eb52cc")
(setf (get pkg_netproxy::UavStatus :datatype-) "pkg_netproxy/UavStatus")
(setf (get pkg_netproxy::UavStatus :definition-)
      "std_msgs/Header header
#位姿信息
float32 roll
float32 pitch
float32 yaw

#GPS位置
float64 latitude
float64 longitude
float64 altitude

#气压计高度
float32 baro_altitude
#激光测高
float32 tf_altitude

#三轴速度
float32 velocity_n
float32 velocity_e
float32 velocity_d

#目标点信息
float64 g_latitude
float64 g_longitude
float64 g_altitude

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :pkg_netproxy/UavStatus "1e2778f4d8e4263c2ff410b965eb52cc")


