;; Auto-generated. Do not edit!


(when (boundp 'pkg_netproxy::PathMsg)
  (if (not (find-package "PKG_NETPROXY"))
    (make-package "PKG_NETPROXY"))
  (shadow 'PathMsg (find-package "PKG_NETPROXY")))
(unless (find-package "PKG_NETPROXY::PATHMSG")
  (make-package "PKG_NETPROXY::PATHMSG"))

(in-package "ROS")
;;//! \htmlinclude PathMsg.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pkg_netproxy::PathMsg
  :super ros::object
  :slots (_header _hb30 _path_ok _min_dis _path_points ))

(defmethod pkg_netproxy::PathMsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:hb30 __hb30) 0)
    ((:path_ok __path_ok) 0)
    ((:min_dis __min_dis) 0.0)
    ((:path_points __path_points) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _hb30 (round __hb30))
   (setq _path_ok (round __path_ok))
   (setq _min_dis (float __min_dis))
   (setq _path_points __path_points)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:hb30
   (&optional __hb30)
   (if __hb30 (setq _hb30 __hb30)) _hb30)
  (:path_ok
   (&optional __path_ok)
   (if __path_ok (setq _path_ok __path_ok)) _path_ok)
  (:min_dis
   (&optional __min_dis)
   (if __min_dis (setq _min_dis __min_dis)) _min_dis)
  (:path_points
   (&rest __path_points)
   (if (keywordp (car __path_points))
       (send* _path_points __path_points)
     (progn
       (if __path_points (setq _path_points (car __path_points)))
       _path_points)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _hb30
    4
    ;; int32 _path_ok
    4
    ;; float32 _min_dis
    4
    ;; geometry_msgs/Point32[] _path_points
    (apply #'+ (send-all _path_points :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _hb30
       (write-long _hb30 s)
     ;; int32 _path_ok
       (write-long _path_ok s)
     ;; float32 _min_dis
       (sys::poke _min_dis (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/Point32[] _path_points
     (write-long (length _path_points) s)
     (dolist (elem _path_points)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _hb30
     (setq _hb30 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _path_ok
     (setq _path_ok (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _min_dis
     (setq _min_dis (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/Point32[] _path_points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _path_points (let (r) (dotimes (i n) (push (instance geometry_msgs::Point32 :init) r)) r))
     (dolist (elem- _path_points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get pkg_netproxy::PathMsg :md5sum-) "8544cf96a4ba5e5ad61125bfbe08010c")
(setf (get pkg_netproxy::PathMsg :datatype-) "pkg_netproxy/PathMsg")
(setf (get pkg_netproxy::PathMsg :definition-)
      "# 时间戳
std_msgs/Header header
# 是否有障碍（0=无，1=有）
int32 hb30
# 是否有路径（0=无，1=有）
int32 path_ok
# 与障碍物的最小距离
float32 min_dis
# 路径点数组（经纬度）
geometry_msgs/Point32[] path_points


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

================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
float32 z
")



(provide :pkg_netproxy/PathMsg "8544cf96a4ba5e5ad61125bfbe08010c")


