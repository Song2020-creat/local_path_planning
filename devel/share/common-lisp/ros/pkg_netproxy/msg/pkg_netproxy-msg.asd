
(cl:in-package :asdf)

(defsystem "pkg_netproxy-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "PathMsg" :depends-on ("_package_PathMsg"))
    (:file "_package_PathMsg" :depends-on ("_package"))
    (:file "UavStatus" :depends-on ("_package_UavStatus"))
    (:file "_package_UavStatus" :depends-on ("_package"))
  ))