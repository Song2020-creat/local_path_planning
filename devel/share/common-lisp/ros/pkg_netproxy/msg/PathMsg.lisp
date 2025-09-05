; Auto-generated. Do not edit!


(cl:in-package pkg_netproxy-msg)


;//! \htmlinclude PathMsg.msg.html

(cl:defclass <PathMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (hb30
    :reader hb30
    :initarg :hb30
    :type cl:integer
    :initform 0)
   (path_ok
    :reader path_ok
    :initarg :path_ok
    :type cl:integer
    :initform 0)
   (min_dis
    :reader min_dis
    :initarg :min_dis
    :type cl:float
    :initform 0.0)
   (path_points
    :reader path_points
    :initarg :path_points
    :type (cl:vector geometry_msgs-msg:Point32)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point32 :initial-element (cl:make-instance 'geometry_msgs-msg:Point32))))
)

(cl:defclass PathMsg (<PathMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PathMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PathMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pkg_netproxy-msg:<PathMsg> is deprecated: use pkg_netproxy-msg:PathMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PathMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pkg_netproxy-msg:header-val is deprecated.  Use pkg_netproxy-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'hb30-val :lambda-list '(m))
(cl:defmethod hb30-val ((m <PathMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pkg_netproxy-msg:hb30-val is deprecated.  Use pkg_netproxy-msg:hb30 instead.")
  (hb30 m))

(cl:ensure-generic-function 'path_ok-val :lambda-list '(m))
(cl:defmethod path_ok-val ((m <PathMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pkg_netproxy-msg:path_ok-val is deprecated.  Use pkg_netproxy-msg:path_ok instead.")
  (path_ok m))

(cl:ensure-generic-function 'min_dis-val :lambda-list '(m))
(cl:defmethod min_dis-val ((m <PathMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pkg_netproxy-msg:min_dis-val is deprecated.  Use pkg_netproxy-msg:min_dis instead.")
  (min_dis m))

(cl:ensure-generic-function 'path_points-val :lambda-list '(m))
(cl:defmethod path_points-val ((m <PathMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pkg_netproxy-msg:path_points-val is deprecated.  Use pkg_netproxy-msg:path_points instead.")
  (path_points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PathMsg>) ostream)
  "Serializes a message object of type '<PathMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'hb30)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'path_ok)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'min_dis))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'path_points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'path_points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PathMsg>) istream)
  "Deserializes a message object of type '<PathMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'hb30) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'path_ok) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_dis) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'path_points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'path_points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point32))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PathMsg>)))
  "Returns string type for a message object of type '<PathMsg>"
  "pkg_netproxy/PathMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PathMsg)))
  "Returns string type for a message object of type 'PathMsg"
  "pkg_netproxy/PathMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PathMsg>)))
  "Returns md5sum for a message object of type '<PathMsg>"
  "8544cf96a4ba5e5ad61125bfbe08010c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PathMsg)))
  "Returns md5sum for a message object of type 'PathMsg"
  "8544cf96a4ba5e5ad61125bfbe08010c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PathMsg>)))
  "Returns full string definition for message of type '<PathMsg>"
  (cl:format cl:nil "# 时间戳~%std_msgs/Header header~%# 是否有障碍（0=无，1=有）~%int32 hb30~%# 是否有路径（0=无，1=有）~%int32 path_ok~%# 与障碍物的最小距离~%float32 min_dis~%# 路径点数组（经纬度）~%geometry_msgs/Point32[] path_points~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PathMsg)))
  "Returns full string definition for message of type 'PathMsg"
  (cl:format cl:nil "# 时间戳~%std_msgs/Header header~%# 是否有障碍（0=无，1=有）~%int32 hb30~%# 是否有路径（0=无，1=有）~%int32 path_ok~%# 与障碍物的最小距离~%float32 min_dis~%# 路径点数组（经纬度）~%geometry_msgs/Point32[] path_points~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PathMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'path_points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PathMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'PathMsg
    (cl:cons ':header (header msg))
    (cl:cons ':hb30 (hb30 msg))
    (cl:cons ':path_ok (path_ok msg))
    (cl:cons ':min_dis (min_dis msg))
    (cl:cons ':path_points (path_points msg))
))
