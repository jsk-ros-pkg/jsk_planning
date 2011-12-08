
(defun move-to-room (goal room &optional (retry 3))
  (let ()
    (send *pr2* :move-to goal :world)
    (send *ri* :move-to (send *pr2* :coords) :frame-id room :retry retry)
;    (send *pr2* :move-to (send *ri* :state :worldcoords "/eng2/7f/73B2") :world)
    ))

(defun pick (obj &key (arm :larm) (use-torso nil))
  ;;(send* *pr2* :inverse-kinematics (send (send coords :copy-worldcoords) :translate #f(-100 0 0)) :move-arm arm args)
  ;;(send *ri* :angle-vector (send *pr2* :angle-vector))
  (ros::spin-once)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :head :look-at (send obj :worldpos) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *ri* :publish-joint-state)
;;  (unix::sleep 5)
  ;;(send *ri* :move-end-coords-plan (send *pr2* :transformation (send (send obj :copy-worldcoords) :translate #f(-100 0 0) :local)) :move-target arm :use-torso use-torso)
  (send *pr2* :inverse-kinematics (send *pr2* :transformation (send (send obj :copy-worldcoords) :translate #f(-100 0 0) :local)) :move-arm arm :use-torso use-torso)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *ri* :move-gripper arm 0.08 :wait t)
  (ros::spin-once)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :inverse-kinematics obj :move-arm arm :use-torso use-torso :rotation-axis t)
  (send *pr2* arm :end-coords :assoc obj)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *ri* :start-grasp arm)
  )

(defun pick2 (pos &key (arm :larm) (use-torso nil))
  (let ((cds (make-coords :pos (v+ pos (float-vector 0 0 51)))))
    (ros::spin-once)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send *ri* :move-end-coords-plan (send (send cds :copy-worldcoords) :translate (float-vector -100 0 0) :local) :move-target arm :use-torso use-torso)
    (send *ri* :wait-interpolation)
    (send *ri* :move-gripper arm 0.08 :wait t)
    (ros::spin-once)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (print (send *pr2* :angle-vector))
    (print (send *pr2* :worldcoords))
    (print (send (send *pr2* :worldcoords) :transform cds))
    (send *pr2* :inverse-kinematics (send (send *pr2* :worldcoords) :transform (send (send cds :copy-worldcoords) :translate (float-vector 30 0 0))) :move-arm arm :use-torso use-torso :rotation-axis t)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (send *ri* :start-grasp arm)
    ))

(defun release (coords &rest args &key (arm :larm) (use-torso nil) &allow-other-keys)
  (let ((obj (car (send *pr2* arm :end-coords :descendants))))
    (ros::spin-once)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send *pr2* :head :look-at (send coords :worldpos) :world)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (send* *pr2* :inverse-kinematics coords :move-arm arm args)
    (send *pr2* arm :end-coords :dissoc obj)
    ;;(send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :publish-joint-state)
;;    (unix::sleep 5)
    (if (null (send *ri* :move-end-coords-plan (send *pr2* :transformation (send *pr2* arm :end-coords)) :move-target arm :use-torso use-torso))
	(send *ri* :angle-vector (send *pr2* :angle-vector)))
    (send *ri* :wait-interpolation)
    (send *ri* :stop-grasp arm)
    ;;    (send *ri* :move-gripper arm 0.08 :wait t)
    (unix::sleep 1)
    (send *ri* :move-gripper arm 0.005 :wait nil)))

(defun hold-chair0 (chair)
  (send *ri* :move-gripper :larm 0.08 :wait nil)
  (send *ri* :move-gripper :rarm 0.08 :wait nil)
  (send *pr2* :larm :inverse-kinematics (send (send (send (send (send chair :handle-larm) :copy-worldcoords) :translate #f(0 0 -100)) :rotate -1.4 :z) :translate #f(-80 0 0)))
  (send *pr2* :rarm :inverse-kinematics (send (send (send (send (send chair :handle-rarm) :copy-worldcoords) :translate #f(0 0 -100)) :rotate 1.4 :z) :translate #f(-80 0 0)))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *pr2* :larm :inverse-kinematics (send (send (send (send chair :handle-larm) :copy-worldcoords) :translate #f(0 0 -100)) :rotate -1.4 :z))
  (send *pr2* :rarm :inverse-kinematics (send (send (send (send chair :handle-rarm) :copy-worldcoords) :translate #f(0 0 -100)) :rotate 1.4 :z))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *ri* :move-gripper :larm 0.005 :wait nil)
  (send *ri* :move-gripper :rarm 0.005 :wait t))

(defun hold-chair1 (chair)
  (send *pr2* :larm :inverse-kinematics (send (send (send chair :handle-larm) :copy-worldcoords) :translate #f(100 -70 200)))
  (send *pr2* :rarm :inverse-kinematics (send (send (send chair :handle-rarm) :copy-worldcoords) :translate #f(100 70 200)))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *pr2* :larm :inverse-kinematics (send (send (send chair :handle-larm) :copy-worldcoords) :translate #f(100 -70 0)))
  (send *pr2* :rarm :inverse-kinematics (send (send (send chair :handle-rarm) :copy-worldcoords) :translate #f(100 70 0)))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation))

(defun hold-chair2 (chair)
  (send *pr2* :reset-pose)
  (send *pr2* :larm :inverse-kinematics (send (send (send (send chair :handle-larm) :copy-worldcoords) :rotate -pi/2 :x) :translate #f(30 100 180)))
  (send *pr2* :rarm :inverse-kinematics (send (send (send (send chair :handle-rarm) :copy-worldcoords) :rotate pi/2 :x) :translate #f(30 -100 180)))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *pr2* :larm :inverse-kinematics (send (send (send (send chair :handle-larm) :copy-worldcoords) :rotate -pi/2 :x) :translate #f(30 100 -50)))
  (send *pr2* :rarm :inverse-kinematics (send (send (send (send chair :handle-rarm) :copy-worldcoords) :rotate pi/2 :x) :translate #f(30 -100 -50)))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation))

(defun hold-chair3 (chair)
  ;; (send *ri* :publish-joint-state)
  ;; (send *ri* :move-end-coords-plan (send *pr2* :transformation (send (send (send (send (send chair :handle-larm) :copy-worldcoords) :rotate -pi/2 :x) :translate #f(30 100 180)) :rotate 0.1 :y)) :move-target :larm :use-torso nil)
  ;; (send *ri* :wait-interpolation)
  ;; (send *ri* :publish-joint-state)
  ;; (send *ri* :move-end-coords-plan (send *pr2* :transformation (send (send (send (send (send chair :handle-rarm) :copy-worldcoords) :rotate pi/2 :x) :translate #f(30 -100 180)) :rotate -0.1 :y)) :move-target :rarm :use-torso nil)
  ;; (send *ri* :wait-interpolation)
  (send *pr2* :larm :angle-vector #f(25 0 0 -130 0 0 0))
  (send *pr2* :rarm :angle-vector #f(-25 0 0 -130 0 0 0))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *pr2* :reset-pose)
  (send-message *pr2* euscollada-robot
  		:inverse-kinematics (send (send (send (send chair :handle-larm) :copy-worldcoords) :rotate -pi/2 :x) :translate #f(30 100 200))
  		:rthre (deg2rad 5)
  		:thre 10
  		:stop 300
  		:weight #f(0 1 1 1 1 1 1 0)
  		:link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
  		:move-target (send *pr2* :larm :end-coords)
  		:rotation-axis :y)
  (send-message *pr2* euscollada-robot
  		:inverse-kinematics (send (send (send (send chair :handle-rarm) :copy-worldcoords) :rotate pi/2 :x) :translate #f(30 -100 200))
  		:rthre (deg2rad 5)
  		:thre 10
  		:stop 300
  		:weight #f(0 1 1 1 1 1 1 0)
  		:link-list (send *pr2* :link-list (send (send *pr2* :rarm :end-coords) :parent))
  		:move-target (send *pr2* :rarm :end-coords)
  		:rotation-axis :y)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send-message *pr2* euscollada-robot
		:inverse-kinematics (send (send (send (send chair :handle-larm) :copy-worldcoords) :rotate -pi/2 :x) :translate #f(30 100 -120))
		:rthre (deg2rad 5)
		:thre 10
		:stop 300
		:weight #f(0 1 1 1 1 1 1 0)
		:link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
		:move-target (send *pr2* :larm :end-coords)
		:rotation-axis :y)
  (send-message *pr2* euscollada-robot
		:inverse-kinematics (send (send (send (send chair :handle-rarm) :copy-worldcoords) :rotate pi/2 :x) :translate #f(30 -100 -120))
		:rthre (deg2rad 5)
		:thre 10
		:stop 300
		:weight #f(0 1 1 1 1 1 1 0)
		:link-list (send *pr2* :link-list (send (send *pr2* :rarm :end-coords) :parent))
		:move-target (send *pr2* :rarm :end-coords)
		:rotation-axis :y)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation))

(defun hold-chair4 (chair)
  (hold-chair3 chair)
;;  (send *pr2* :larm :end-coords :assoc chair)
;;  (send *ri* :start-grasp :larm)
  (send *pr2* :inverse-kinematics (send (send *pr2* :larm :end-coords :copy-worldcoords) :translate #f(-300 0 0) *pr2*) :move-arm :larm :use-torso nil :rotation-axis :y)
  (send *pr2* :inverse-kinematics (send (send *pr2* :rarm :end-coords :copy-worldcoords) :translate #f(-300 0 0) *pr2*) :move-arm :rarm :use-torso nil :rotation-axis :y)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *pr2* :rarm :angle-vector #f(-14.3227 16.0059 -85.3758 -102.121 -35.757 -32.3901 -173.201))
  (send *pr2* :larm :angle-vector #f(8.64394 19.7097 109.218 -85.0018 43.0966 -7.98218 163.739))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation))

(defun release-chair0 (chair)
  (send *ri* :move-gripper :larm 0.08 :wait nil)
  (send *ri* :move-gripper :rarm 0.08 :wait t)
  (send *pr2* :larm :inverse-kinematics (send (send (send (send (send chair :handle-larm) :copy-worldcoords) :translate #f(0 0 -100)) :rotate -1.4 :z) :translate #f(-40 0 0)))
  (send *pr2* :rarm :inverse-kinematics (send (send (send (send (send chair :handle-rarm) :copy-worldcoords) :translate #f(0 0 -100)) :rotate 1.4 :z) :translate #f(-40 0 0)))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (unix::sleep 1)
  (send *pr2* :larm :inverse-kinematics (send (send (send (send (send chair :handle-larm) :copy-worldcoords) :translate #f(0 0 -100)) :rotate -1.4 :z) :translate #f(-80 0 0)))
  (send *pr2* :rarm :inverse-kinematics (send (send (send (send (send chair :handle-rarm) :copy-worldcoords) :translate #f(0 0 -100)) :rotate 1.4 :z) :translate #f(-80 0 0)))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *ri* :move-gripper :larm 0.005 :wait nil)
  (send *ri* :move-gripper :rarm 0.005 :wait nil))

(defun release-chair1 (chair)
  (send *pr2* :larm :inverse-kinematics (send (send (send chair :handle-larm) :copy-worldcoords) :translate #f(100 -70 50)))
  (send *pr2* :rarm :inverse-kinematics (send (send (send chair :handle-rarm) :copy-worldcoords) :translate #f(100 70 50)))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (unix::sleep 1)
  (send *pr2* :larm :inverse-kinematics (send (send (send chair :handle-larm) :copy-worldcoords) :translate #f(100 -70 200)))
  (send *pr2* :rarm :inverse-kinematics (send (send (send chair :handle-rarm) :copy-worldcoords) :translate #f(100 70 200)))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation))

(defun release-chair2 (chair)
  (send *pr2* :larm :inverse-kinematics (send (send (send (send chair :handle-larm) :copy-worldcoords) :rotate -pi/2 :x) :translate #f(30 100 80)))
  (send *pr2* :rarm :inverse-kinematics (send (send (send (send chair :handle-rarm) :copy-worldcoords) :rotate pi/2 :x) :translate #f(30 -100 80)))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (unix::sleep 1)
  (send *pr2* :larm :inverse-kinematics (send (send (send (send chair :handle-larm) :copy-worldcoords) :rotate -pi/2 :x) :translate #f(30 100 180)))
  (send *pr2* :rarm :inverse-kinematics (send (send (send (send chair :handle-rarm) :copy-worldcoords) :rotate pi/2 :x) :translate #f(30 -100 180)))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation))

(defun release-chair3 (chair)
  (send-message *pr2* euscollada-robot
		:inverse-kinematics (send (send (send (send chair :handle-larm) :copy-worldcoords) :rotate -pi/2 :x) :translate #f(30 100 60))
		:rthre (deg2rad 5)
		:thre 10
		:stop 300
		:weight #f(0 1 1 1 1 1 1 0)
		:link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
		:move-target (send *pr2* :larm :end-coords)
		:rotation-axis :y)
  (send-message *pr2* euscollada-robot
		:inverse-kinematics (send (send (send (send chair :handle-rarm) :copy-worldcoords) :rotate pi/2 :x) :translate #f(30 -100 60))
		:rthre (deg2rad 5)
		:thre 10
		:stop 300
		:weight #f(0 1 1 1 1 1 1 0)
		:link-list (send *pr2* :link-list (send (send *pr2* :rarm :end-coords) :parent))
		:move-target (send *pr2* :rarm :end-coords)
		:rotation-axis :y)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (unix::sleep 1)
  (send-message *pr2* euscollada-robot
		:inverse-kinematics (send (send (send (send chair :handle-larm) :copy-worldcoords) :rotate -pi/2 :x) :translate #f(30 100 180))
		:rthre (deg2rad 5)
		:thre 10
		:stop 300
		:weight #f(0 1 1 1 1 1 1 0)
		:link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
		:move-target (send *pr2* :larm :end-coords)
		:rotation-axis :y)
  (send-message *pr2* euscollada-robot
		:inverse-kinematics (send (send (send (send chair :handle-rarm) :copy-worldcoords) :rotate pi/2 :x) :translate #f(30 -100 180))
		:rthre (deg2rad 5)
		:thre 10
		:stop 300
		:weight #f(0 1 1 1 1 1 1 0)
		:link-list (send *pr2* :link-list (send (send *pr2* :rarm :end-coords) :parent))
		:move-target (send *pr2* :rarm :end-coords)
		:rotation-axis :y)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation))

(defun release-chair4 (chair)
  (send *pr2* :translate #f(500 0 0))
  (send *ri* :go-pos-unsafe 0.5 0)
;;  (send *pr2* :larm :end-coords :dissoc chair)
;;  (send *ri* :stop-grasp :larm)
  (send *pr2* :inverse-kinematics (send (send *pr2* :larm :end-coords :copy-worldcoords) :translate #f(300 300 0) *pr2*) :move-arm :larm :use-torso nil :rotation-axis :y)
  (send *pr2* :inverse-kinematics (send (send *pr2* :rarm :end-coords :copy-worldcoords) :translate #f(300 -300 0) *pr2*) :move-arm :rarm :use-torso nil :rotation-axis :y)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (unix::sleep 1)
  (send *pr2* :reset-pose)
  (send *pr2* :inverse-kinematics (send (send *pr2* :larm :end-coords :copy-worldcoords) :translate #f(0 200 0) *pr2*) :move-arm :larm :use-torso nil :rotation-axis nil)
  (send *pr2* :inverse-kinematics (send (send *pr2* :rarm :end-coords :copy-worldcoords) :translate #f(0 -200 0) *pr2*) :move-arm :rarm :use-torso nil :rotation-axis nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (unix::sleep 1)
  (send *pr2* :translate #f(-500 0 0))
  (send *ri* :go-pos-unsafe -0.5 0)
  )

(defun move-chair (chair x y &optional (d 0) &key (direct :x))
  (let ((cds-list '(nil nil)) (rc-chair-pos nil))
    ;;(change-handle-num 1)
   ;; (send *ri* :go-pos-unsafe 0.2 0 0)
    (send *pr2* :head :look-at (vector-mean (send-all (send chair :handle) :worldpos)) :world)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    ;; (while (some #'null cds-list)
    ;;   (setq cds-list (get-handle-coords-list-from-handle-detector 2 "base_link") *tfl* (ros::time-now))
    ;;   (print cds-list))
    ;; (setq rc-chair-pos (vector-mean (send-all cds-list :worldpos)))
    ;; ;; (while (null rc-chair-pos)
    ;; ;;   (setq rc-chair-pos (get-handle-coords-from-handle-detector 0 1 "base_link" (ros::time-now) 5.0)))
    ;; (send *ri* :go-pos (- (elt rc-chair-pos 0) 200) (elt rc-chair-pos 1))
    (hold-chair4 chair)
    (send *pr2* :larm :end-coords :assoc chair)
    (send *pr2* :translate (float-vector (* x 1000) (* y 1000) 0))
    (send *pr2* :rotate (deg2rad d) :z)
   ;; (send *ri* :go-pos-unsafe -0.2 0 0)
    (if (eq direct :x)
	(progn
	  (send *ri* :go-pos-unsafe x 0 0)
	  (send *ri* :go-pos-unsafe 0 y d))
      (progn
	(send *ri* :go-pos-unsafe 0 y d)
	(send *ri* :go-pos-unsafe x 0 0)))
    (send *pr2* :larm :end-coords :dissoc chair)
    (release-chair4 chair)
    ;;(send *ri* :move-gripper :larm 0.005 :wait nil)
    ))

(defun wipe-table4 (&rest args)
  (let (avs model check-avs (wipe-rot #2f((0.965926 5.682722e-21 0.258817) (-8.512166e-16 1.0 3.176784e-15) (-0.258817 -3.288848e-15 0.965926))))
    (initial-wipe-motion :height 650)
    (setq model (make-cube 1000 1000 20))
    (send model :set-color #f(1 1 1 0))
    (send model :move-to (send *pr2* :worldcoords) :world)
    (send model :locate (float-vector 700 0 (- (elt (send *pr2* :larm :end-coords :worldpos) 2) 30)))
;;    (objects (list *pr2* model))
    (setq avs (wipe-motion3 :offset #f(-200 0 0) :width 700 :init t :model model :arm :larm))
    (dotimes (i 10)
      (setq check-avs (apply #'wipe-motion3 :offset #f(50 0 0) :rot wipe-rot :width 700 :model model :arm :larm args))
      (unless check-avs
	(return nil))
      (setq avs (append avs check-avs)))
;;    (setq *flag* (instance std_msgs::Bool :init))
;;    (send *flag* :data t)
;;    (ros::publish "wipe_table" *flag*)
    (send *ri* :angle-vector-sequence avs (make-sequence 'list (length avs) :initial-element 300))
    (send *ri* :wait-interpolation)
;;    (send *flag* :data nil)
;;    (ros::publish "wipe_table" *flag*)

    (send *pr2* :reset-manip-pose)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (pr2-tuckarm-pose :larm)
    (ros::spin-once)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send *pr2* :torso :waist-z :joint-angle 0)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
  ))

(defun initial-wipe-motion (&key (height 700) (base-tm 2000))
    ;; (send *pr2* :reset-pose)
    ;; (send *irtviewer* :draw-objects)
    ;; (send *ri* :angle-vector (send *pr2* :angle-vector) base-tm)
    ;; (send *ri* :wait-interpolation)

    ;; (send *pr2* :larm :shoulder-p :joint-angle -30)
    ;; (send *irtviewer* :draw-objects)
    ;; (send *ri* :angle-vector (send *pr2* :angle-vector) base-tm)
    ;; (send *ri* :wait-interpolation)

    ;;    (send *pr2* :larm :angle-vector #f(0.0 -30.0 180.0 -60.0 0.0 -50.0 0.0))
    (ros::spin-once)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
;;    (send *pr2* :head :angle-vector #f(0 35))
  ;;  (send *ri* :angle-vector (send *pr2* :angle-vector))
    (catch-duster2)

    (ros::spin-once)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))

    (send *pr2* :torso :waist-z :joint-angle (/ height 5))
 ;;   (send *pr2* :larm :angle-vector #f(0.0 -50.0 180.0 -60.0 180.0 -15.0 0.0))

;;    (send *ri* :angle-vector (send *pr2* :angle-vector) base-tm)

    ;;    (send *pr2* :larm :angle-vector #f(15.0 -50.0 180.0 -70.0 180.0 -15.0 0.0))
    ;;(send *ri* :angle-vector-motion-plan (send *pr2* :angle-vector) (* 10 base-tm) :move-target :larm :use-torso t)

   ;; (setq a (send *ri* :angle-vector-motion-plan (send *pr2* :angle-vector) :move-target :larm :use-torso t))
    ;;(send *ri* :wait-interpolation)

    ;;  (send *pr2* :angle-vector sisei)
    ;;  (send *pr2* :torso :waist-z :joint-angle (elt (car (reverse a)) 0))

    ;;  (send *ri* :angle-vector (send *pr2* :angle-vector) base-tm)
    ;;  (send *ri* :wait-interpolation)
    ;;  (unix::sleep (ceiling (/ (abs (- (elt (car (reverse a)) 0) (elt sisei 0))) 13)))
    (print "move torso finish")

    ;;  (dotimes (i (length a))
    ;;	  (setf (elt (elt a i) 0) (elt (car (reverse a)) 0)))
    ;;  (send *ri* :angle-vector-sequence a (make-sequence 'list (length a) :initial-element (/ (* 3 base-tm) (length a))))
    ;;  (send *ri* :wait-interpolation)
    ;; (send *pr2* :angle-vector sisei)
    ;; (send *pr2* :torso :waist-z :joint-angle (/ height 5))
    ;; (send *pr2* :larm :angle-vector #f(0.0 -50.0 180.0 -60.0 180.0 -15.0 0.0))
    ;; (send *ri* :angle-vector-motion-plan (send *pr2* :angle-vector) (* 10 base-tm) :move-target :larm :use-torso nil)
    ;; (send *irtviewer* :draw-objects)
    ;; (send *ri* :wait-interpolation)
    ;(ros::spin-once)
    ;(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send *pr2* :larm :angle-vector #f(0.0 -50.0 180.0 -60.0 180.0 -15.0 0.0))
    (send *ri* :angle-vector (send *pr2* :angle-vector) (* 3 base-tm))
   ;; (send *ri* :viewer :draw-objects)
    (send *ri* :wait-interpolation)
    (print "move-arm finish")
    (send *pr2* :larm :inverse-kinematics
	  (send (send *pr2* :larm :end-coords :copy-worldcoords) :translate
		(float-vector 0 0
			      (- height (elt (send *pr2* :larm :end-coords :worldpos) 2))))
	  :rotation-axis t :revert-if-fail nil)

    (send *pr2* :look-at-hand :larm)
    (send *ri* :angle-vector (send *pr2* :angle-vector) (* 3 base-tm))
    (send *ri* :wait-interpolation)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))

    (print "set arm on table")
    (ros::spin-once)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send *pr2* :look-at-hand :larm)
   ;; (send *ri* :viewer :draw-objects)
    (send *ri* :angle-vector (send *pr2* :angle-vector) base-tm)
    (send *ri* :wait-interpolation))

(defun wipe-motion3
  (&key (offset #f(0 0 0)) (width 500) (debug t) (rot nil)
	(init nil) (model nil) (arm :larm)
	(avoid-collision-null-gain 100)
	(avoid-collision-joint-gain 0.0)
	(debug-view nil)
	(stop 300)
	(rotation-axis :z)
	(division 20))
  (let (coords orig-coords (coords-list nil) avs (check-links (list (elt (send *pr2* arm :links) 2) (elt (send *pr2* arm :links) 3) (elt (send *pr2* arm :links) 4))) (distance-threshold 10))

    (setq avs (offset-endcoords :offset offset :model model :arm arm
				:avoid-collision-null-gain avoid-collision-null-gain
				:avoid-collision-joint-gain avoid-collision-joint-gain
				:rot rot
				:stop stop
				:debug-view debug-view
				:rotation-axis rotation-axis))

    (unless avs
      (print "fail")
      (return-from wipe-motion3 nil))
    (nreverse avs)

    (if (null rot)
	(setq orig-coords (send *pr2* arm :end-coords :copy-worldcoords))
      (setq orig-coords (make-coords :pos (send (send *pr2* arm :end-coords :copy-worldcoords) :worldpos) :rot rot)))

    (cond
     (init
      (dotimes (i (/ division 2))
	(setq coords (send orig-coords :copy-worldcoords))
	(if (eq arm :larm)
	    (send coords :translate (float-vector 0 (* (/ width 2) (sin (/ (* pi/2 (1+ i)) division))) 0) :world)
	  (send coords :translate (float-vector 0 (* (/ width 2) (sin (/ (* -pi/2 (1+ i))  division))) 0) :world))
	(push coords coords-list)))

     (t
      (dotimes (i division)
	(setq coords (send orig-coords :copy-worldcoords))
	(if (eq arm :larm)
	    (send coords :translate  (float-vector 0 (* width (/ (1- (cos (/ (* 2pi i) (1- division)))) 2)) 0) :world)
	  (send coords :translate  (float-vector 0 (- (* width (/ (1- (cos (/ (* 2pi i) (1- division)))) 2))) 0) :world))
	(push coords coords-list))))

    (dolist (i (reverse coords-list))
      (when (send-message *pr2* euscollada-robot
			  :inverse-kinematics i
			  :rthre (deg2rad 5)
			  :thre 10
			  :stop stop
			  :weight #f(0 1 1 1 1 1 1 0)
			  :link-list (send *pr2* :link-list (send (send *pr2* arm :end-coords) :parent))
			  :move-target (send *pr2* arm :end-coords)
			  :rotation-axis rotation-axis
			  :avoid-collision-null-gain avoid-collision-null-gain
			  :avoid-collision-joint-gain avoid-collision-joint-gain
			  :debug-view debug-view
			  :collision-avoidance-link-pair
			  (mapcar #'(lambda(x) (list x model)) check-links)
			  :additional-check
			  #'(lambda()
			      (x::window-main-one)
			      (not
			       (some #'null
				     (mapcar #'(lambda (x) (> (car (pqp-collision-distance x model)) distance-threshold)) check-links)))))
	(send *pr2* :look-at-hand arm)
	;; (print (send i :difference-position (send *pr2* arm :end-coords)))
	(if debug (send *ri* :viewer :draw-objects))
	(push (send *pr2* :angle-vector) avs)))
    (reverse avs)))

(defun offset-endcoords (&key (offset #f(0 0 0)) (rot nil)
			      (model nil) (arm :larm) (debug t)
			      (avoid-collision-null-gain 100)
			      (avoid-collision-joint-gain 0.0)
			      (stop 50)
			      (debug-view nil)
			      (rotation-axis :z)
			      (division nil)
			      (look t))
  (let (coords orig-coords (coords-list nil) (avs nil) check-links (list (elt (send *pr2* arm :links) 2) (elt (send *pr2* arm :links) 3) (elt (send *pr2* arm :links) 4)) (distance-threshold 10))

    (when (null division)
      (setq division (ceiling (/ (norm offset) 50)))
      (if (= division 0)
	  (setq division 1)))


    (if (null rot)
	(setq orig-coords (send *pr2* arm :end-coords :copy-worldcoords))
      (setq orig-coords (make-coords :pos (send (send *pr2* arm :end-coords :copy-worldcoords) :worldpos) :rot rot)))

  (dotimes (i division)
    (setq coords (send orig-coords :copy-worldcoords))
    (push
     (send coords :translate (scale (sin (/ (* pi/2 (1+ i)) division)) offset) :world) coords-list))

  (dolist (i (reverse coords-list))
    (when (send-message *pr2* euscollada-robot
			:inverse-kinematics i
			:rthre (deg2rad 5)
			:thre 10
			:stop stop
			:weight #f(0 1 1 1 1 1 1 0)
			:link-list (send *pr2* :link-list (send (send *pr2* arm :end-coords) :parent))
			:move-target (send *pr2* arm :end-coords)
			:rotation-axis rotation-axis
			:avoid-collision-null-gain avoid-collision-null-gain
			:avoid-collision-joint-gain avoid-collision-joint-gain
			:debug-view debug-view
			:collision-avoidance-link-pair
			(mapcar #'(lambda(x) (list x model)) check-links)
			:additional-check
			#'(lambda()
			    (x::window-main-one)
			    (not
			     (some #'null
				   (mapcar #'(lambda (x) (> (car (pqp-collision-distance x model)) distance-threshold)) check-links)))))
      (if look (send *pr2* :look-at-hand arm))
      (if debug (send *ri* :viewer :draw-objects))
      (push (send *pr2* :angle-vector) avs)))
  (if (= division (length avs))
      (reverse avs)
    nil)))

(defun catch-duster()
  (let ((before-avs
	 '(#f(11.5096 0.003327 64.8541 69.9686 -121.631 50.651 -115.013 160.007 -5.02184 74.1837 -105.019 -89.9853 69.9963 -5.14485 19.9982 31.5426 79.664) #f(11.5096 -0.006174 64.8541 69.9686 -121.631 50.651 -115.013 160.007 -5.02184 74.1837 -105.019 -89.9853 69.9963 -5.14485 19.9982 31.5426 79.664) #f(11.5096 0.098331 64.8541 69.9778 -121.631 50.6543 -115.013 160.007 -5.02184 74.1837 -105.019 -89.977 69.9963 -5.14485 19.9982 31.5426 79.664) #f(11.5096 0.193335 64.951 69.9319 -121.706 50.6576 -115.013 160.007 -5.02659 74.1837 -105.019 -89.977 69.9963 -5.14485 19.9982 31.5426 79.664) #f(11.5096 2.20742 64.8783 69.9135 -121.772 50.8366 -115.013 160.007 -5.03134 74.1788 -105.019 -89.977 69.9963 -5.14485 19.9982 31.5426 79.664) #f(11.5096 7.04312 64.7474 69.9503 -121.681 51.5923 -115.013 160.007 -5.04559 74.1691 -105.019 -89.977 69.9963 -5.14485 19.9982 31.5426 79.664) #f(11.5096 11.3468 64.7717 70.0973 -121.648 52.4209 -115.013 160.007 -5.05034 74.1691 -105.019 -89.977 69.9963 -5.14485 19.9982 31.5426 79.664) #f(11.5096 18.2631 64.9995 70.3178 -121.607 53.3589 -115.013 160.007 -5.05034 74.1691 -105.019 -89.977 69.9963 -5.14485 19.9982 31.5426 79.664) #f(11.5096 25.0654 65.1546 70.6301 -121.607 53.7235 -115.013 160.007 -5.05034 74.1691 -105.019 -89.977 69.9963 -5.14485 19.9982 31.5426 79.664) #f(11.5095 32.5944 65.804 70.4923 -121.698 54.0085 -115.013 160.007 -5.05034 74.1691 -105.019 -89.977 69.9963 -5.14485 19.9982 31.5426 79.664) #f(11.5095 41.1781 66.9043 69.8584 -121.689 54.4327 -115.013 160.007 -5.2166 74.0237 -104.946 -89.9023 69.9963 -5.14485 19.9982 31.5426 79.664) #f(11.5095 49.5004 67.9658 69.3714 -121.548 54.6183 -115.013 160.007 -5.2641 73.9849 -104.936 -89.894 69.9963 -5.14485 19.9982 31.5426 79.664) #f(11.5095 57.7705 69.0612 68.7467 -121.98 55.1884 -115.013 160.007 -5.38761 73.888 -104.762 -89.7779 69.9963 -5.29193 20.1453 31.5426 79.664) #f(11.5095 64.074 69.512 68.4067 -121.673 58.2774 -115.013 160.007 -5.62037 73.6796 -104.615 -89.6452 69.9963 -5.53374 20.3871 31.5426 79.664) #f(11.5095 68.8765 69.6525 68.1403 -120.794 60.7135 -114.392 164.916 -5.82463 73.5584 -104.587 -89.5871 69.9963 -5.59606 20.4494 31.5426 79.664) #f(11.5095 72.8951 69.7398 68.618 -118.811 65.3271 -111.323 176.655 -5.89113 73.5099 -104.569 -89.5623 69.9963 -5.616 20.4694 31.5426 79.664) #f(11.5095 75.3747 69.9579 69.8308 -117.144 69.2746 -108.427 -175.568 -5.91013 73.4857 -104.56 -89.5623 69.9963 -5.62099 20.4793 31.5426 79.664) #f(11.5095 77.8401 71.0 71.9899 -113.536 72.8707 -104.879 -160.493 -5.91963 73.4808 -104.56 -89.554 69.9963 -5.62348 20.4818 31.5426 79.664) #f(11.5095 80.0157 72.6382 74.1031 -110.218 75.847 -101.362 -143.594 -5.92913 73.4808 -104.56 -89.554 69.9963 -5.62597 20.4843 31.5426 79.664) #f(11.5095 82.709 74.5479 75.8671 -106.203 77.0999 -100.841 -130.469 -5.92913 73.4808 -104.56 -89.554 69.9963 -5.62597 20.4843 31.5426 79.664) #f(11.5095 84.6709 74.6206 78.6693 -100.729 77.2822 -100.619 -115.1 -5.92438 73.4808 -104.569 -89.554 69.9963 -5.62597 20.4843 31.5426 79.664) #f(11.5095 86.4617 74.6836 81.3613 -94.914 77.259 -102.152 -106.827 -5.92438 73.4808 -104.569 -89.554 69.9963 -5.62597 20.4843 31.5426 79.664) #f(11.5095 89.3641 74.8242 86.975 -88.5685 77.4379 -102.783 -100.382 -5.91963 73.4808 -104.569 -89.554 69.9963 -5.62597 20.4843 31.5426 79.664) #f(11.5095 91.7392 74.8484 90.8797 -81.5926 80.0364 -103.159 -91.8892 -5.91013 73.4808 -104.569 -89.554 69.9963 -5.62597 20.4843 31.5426 79.664) #f(11.5095 93.1642 74.9017 94.2884 -75.8776 80.4076 -100.507 -81.8379 -5.90538 73.4905 -104.569 -89.554 69.9963 -5.62597 20.4843 31.5426 79.664) #f(11.5095 94.2853 74.892 96.2729 -73.4057 80.4076 -101.556 -70.6075 -5.88638 73.4905 -104.569 -89.554 69.9963 -5.62597 20.4843 31.5426 79.664) #f(11.5095 94.3613 74.703 96.8058 -73.0325 80.4076 -101.556 -70.6075 -5.88638 73.4905 -104.569 -89.554 69.9963 -5.62597 20.4843 31.5426 79.664) #f(11.5095 94.3613 74.64 96.8058 -72.9744 80.4076 -101.447 -71.6346 -5.88638 73.4905 -104.569 -89.554 69.9963 -5.62597 20.4843 31.5426 79.664)))
	(after-avs
	 '(#f(11.5088 106.104 59.6727 115.889 -82.2562 70.4394 -65.599 -78.0583 -7.32569 72.9865 -104.909 -88.8157 69.993 -5.29941 20.0032 45.6092 76.4112) #f(11.5088 106.104 59.6727 115.889 -82.2562 70.4394 -65.599 -78.0583 -7.32569 72.9865 -104.909 -88.8157 69.993 -5.29941 20.0032 45.6092 76.4112) #f(11.5088 106.094 53.1487 116.155 -83.011 70.4394 -65.599 -78.0583 -7.35419 72.9768 -104.918 -88.8157 69.993 -5.29941 20.0032 52.4552 74.8033) #f(11.5088 106.099 46.901 116.146 -84.6617 70.4394 -65.599 -78.0583 -7.35894 72.9719 -104.918 -88.8157 69.993 -5.29941 20.0032 56.6415 72.9613) #f(11.5088 106.094 41.3222 115.714 -90.1943 69.4881 -65.599 -78.0583 -7.37319 72.9671 -104.918 -88.8074 69.993 -5.29941 20.0032 57.4751 71.549) #f(11.5088 106.094 34.5559 114.988 -94.9804 63.0682 -65.599 -78.0583 -7.37319 72.9671 -104.927 -88.8074 69.993 -5.29941 20.0032 58.7375 69.7043) #f(11.5088 106.061 29.8834 114.86 -97.9499 59.1737 -65.599 -78.0583 -7.37794 72.9671 -104.927 -88.8074 69.993 -5.29941 20.0032 62.7997 68.6795) #f(11.5088 105.553 20.4949 113.702 -99.6005 49.857 -65.599 -78.0583 -7.37794 72.9671 -104.927 -88.8074 69.993 -5.29941 20.0032 64.8693 65.4141) #f(11.5088 104.517 15.2651 113.059 -100.372 41.2064 -65.599 -78.0583 -7.37794 72.9671 -104.936 -88.8074 69.993 -5.29941 20.0032 62.5703 63.4979) #f(11.5088 104.441 10.5393 113.151 -103.325 32.2178 -65.599 -78.0583 -7.37794 72.9671 -104.936 -88.8074 69.993 -5.29941 20.0032 62.7488 62.6029) #f(11.5088 104.346 4.97505 113.821 -105.224 25.3835 -65.599 -78.0583 -7.37794 72.9671 -104.936 -88.8074 69.993 -5.29941 20.0032 63.6599 61.4813) #f(11.5088 103.914 -2.17417 113.316 -107.995 19.8087 -65.589 -78.0633 -7.40644 72.9477 -104.946 -88.7991 69.993 -5.29941 20.0032 64.4691 60.103) #f(11.5088 102.636 -7.16651 112.765 -109.115 13.1269 -65.5666 -78.0758 -7.41119 72.9477 -104.946 -88.7991 69.993 -5.29941 20.0032 64.1682 58.6937) #f(11.5088 100.142 -12.1298 112.122 -109.903 2.77933 -65.4245 -78.0783 -7.41119 72.9477 -104.955 -88.7991 69.993 -5.29941 20.0032 60.1354 56.8041) #f(11.5088 100.494 -12.3333 109.319 -113.229 -1.17475 -65.4245 -78.0783 -7.41594 72.9477 -104.955 -88.7991 69.993 -5.29941 20.0032 61.0708 57.1668) #f(11.5088 100.513 -12.2606 105.92 -114.564 -6.70315 -63.8639 -78.0683 -7.42069 72.9428 -104.955 -88.7991 69.993 -5.29941 20.0032 59.4012 55.9563) #f(11.5088 100.665 -12.0813 99.1854 -114.224 -6.38166 -62.8194 -78.0509 -7.42544 72.9428 -104.964 -88.7908 69.993 -5.29941 20.0032 58.3135 53.9058) #f(11.5088 100.271 -11.6063 96.5761 -115.842 -6.65012 -63.4352 -78.193 -7.42544 72.9428 -104.964 -88.7908 69.993 -5.29941 20.0032 56.1609 54.399) #f(11.5088 100.199 -11.2476 92.6805 -115.286 -10.6771 -62.089 -78.4173 -7.44444 72.9283 -104.964 -88.7908 69.993 -5.29941 20.0032 56.5597 52.1163) #f(11.5088 98.8407 -15.542 93.0113 -115.883 -27.5407 -53.3465 -75.1741 -7.46819 72.9186 -104.973 -88.7908 69.993 -5.29941 20.0032 53.417 47.5959) #f(11.5088 94.138 -20.3065 93.1767 -115.891 -32.688 -49.4776 -71.3101 -7.48244 72.9089 -105.028 -88.7743 69.993 -5.29941 20.0032 47.4559 46.4945) #f(11.5088 85.7539 -20.481 93.8014 -115.692 -45.4816 -38.4216 -60.2542 -7.70095 72.8314 -105.203 -88.7494 69.993 -5.29941 20.0032 40.526 43.9182) #f(11.5088 77.0563 -20.4907 93.7187 -114.473 -64.42 -24.5313 -46.3689 -7.73421 72.7926 -105.276 -88.7494 69.993 -5.29941 20.0032 33.8047 41.0685) #f(11.5088 69.1472 -20.515 93.7279 -110.342 -76.6104 -16.7436 -38.5811 -7.74371 72.7926 -105.285 -88.7494 69.993 -5.29941 20.0032 28.5389 38.8975) #f(11.5088 62.592 -20.2387 96.1535 -105.291 -76.7794 -15.9708 -37.8083 -7.75321 72.7926 -105.304 -88.7494 69.993 -5.29941 20.0032 24.6299 37.9168) #f(11.5088 54.9346 -20.3211 101.712 -97.5434 -76.8026 -15.9633 -37.8009 -7.82446 72.7684 -105.423 -88.7328 69.993 -5.29941 20.0032 21.009 36.9949) #f(11.5088 43.9237 -20.3356 106.48 -83.2433 -82.7088 -15.9683 -37.7959 -7.80071 72.6763 -105.46 -88.7245 69.993 -5.29941 20.0032 16.5595 32.9589) #f(11.5088 34.0005 -20.3696 111.497 -68.8436 -85.9768 -15.9483 -37.7759 -7.82446 72.6763 -105.478 -88.7245 69.993 -5.29941 20.0032 13.3715 29.5216) #f(11.5088 25.4216 -20.3405 120.547 -58.8318 -95.8703 -15.9483 -37.7759 -7.82446 72.6763 -105.488 -88.7245 69.993 -5.29941 20.0032 11.0344 28.9091) #f(11.5088 17.5268 -20.3211 129.045 -49.3178 -104.435 -15.9483 -37.7759 -7.81971 72.6763 -105.506 -88.7245 69.993 -5.29941 20.0032 8.88368 27.6934) #f(11.5088 10.3065 -20.3162 139.887 -42.8562 -114.789 -15.9483 -37.7759 -7.82446 72.6763 -105.506 -88.7245 69.993 -5.29941 20.0032 6.75793 27.4731) #f(11.5088 4.58252 -20.3889 152.308 -36.7927 -128.176 -15.9483 -37.7759 -7.82446 72.6763 -105.515 -88.7245 69.993 -5.29941 20.0032 6.081 26.5202) #f(11.5088 -0.6237 -20.3889 165.575 -30.356 -143.535 -15.9359 -37.7784 -7.82446 72.6763 -105.515 -88.7162 69.993 -5.29941 20.0032 5.56225 24.5291) #f(11.5088 -4.77062 -20.3986 175.425 -28.1496 -155.297 -9.40704 -35.6819 -7.82446 72.6714 -105.524 -88.7162 69.993 -5.29941 20.0032 4.54259 24.8059) #f(11.5088 -6.5852 -20.4035 177.023 -26.5736 -155.785 -6.37819 -34.9016 -7.82446 72.6714 -105.524 -88.7162 69.993 -5.29941 20.0032 3.35129 24.6352))))

    ;;(pr2-tuckarm-pose :larm)
    (send *pr2* :head :angle-vector #f(31.5426 79.664))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
    (send *ri* :wait-interpolation)
    (send *ri* :move-gripper :larm 0.09)

    (send *ri* :angle-vector-sequence before-avs (make-sequence 'list (length before-avs) :initial-element 300))
    (send *ri* :wait-interpolation)
    (send *ri* :move-gripper :larm 0.0 :effort 100)
    (send *ri* :angle-vector-sequence after-avs (make-sequence 'list (length after-avs) :initial-element 300))
    (send *ri* :wait-interpolation)))

(defun catch-duster2()
  (let ((dustcds (make-coords :pos #f(206.134 232.7 337.956) :rpy #f(-1.753 0.049 -0.015)))
	(after-avs
	  '(#f(11.5088 106.104 59.6727 115.889 -82.2562 70.4394 -65.599 -78.0583 -7.32569 72.9865 -104.909 -88.8157 69.993 -5.29941 20.0032 45.6092 76.4112) #f(11.5088 106.104 59.6727 115.889 -82.2562 70.4394 -65.599 -78.0583 -7.32569 72.9865 -104.909 -88.8157 69.993 -5.29941 20.0032 45.6092 76.4112) #f(11.5088 106.094 53.1487 116.155 -83.011 70.4394 -65.599 -78.0583 -7.35419 72.9768 -104.918 -88.8157 69.993 -5.29941 20.0032 52.4552 74.8033) #f(11.5088 106.099 46.901 116.146 -84.6617 70.4394 -65.599 -78.0583 -7.35894 72.9719 -104.918 -88.8157 69.993 -5.29941 20.0032 56.6415 72.9613) #f(11.5088 106.094 41.3222 115.714 -90.1943 69.4881 -65.599 -78.0583 -7.37319 72.9671 -104.918 -88.8074 69.993 -5.29941 20.0032 57.4751 71.549) #f(11.5088 106.094 34.5559 114.988 -94.9804 63.0682 -65.599 -78.0583 -7.37319 72.9671 -104.927 -88.8074 69.993 -5.29941 20.0032 58.7375 69.7043) #f(11.5088 106.061 29.8834 114.86 -97.9499 59.1737 -65.599 -78.0583 -7.37794 72.9671 -104.927 -88.8074 69.993 -5.29941 20.0032 62.7997 68.6795) #f(11.5088 105.553 20.4949 113.702 -99.6005 49.857 -65.599 -78.0583 -7.37794 72.9671 -104.927 -88.8074 69.993 -5.29941 20.0032 64.8693 65.4141) #f(11.5088 104.517 15.2651 113.059 -100.372 41.2064 -65.599 -78.0583 -7.37794 72.9671 -104.936 -88.8074 69.993 -5.29941 20.0032 62.5703 63.4979) #f(11.5088 104.441 10.5393 113.151 -103.325 32.2178 -65.599 -78.0583 -7.37794 72.9671 -104.936 -88.8074 69.993 -5.29941 20.0032 62.7488 62.6029) #f(11.5088 104.346 4.97505 113.821 -105.224 25.3835 -65.599 -78.0583 -7.37794 72.9671 -104.936 -88.8074 69.993 -5.29941 20.0032 63.6599 61.4813) #f(11.5088 103.914 -2.17417 113.316 -107.995 19.8087 -65.589 -78.0633 -7.40644 72.9477 -104.946 -88.7991 69.993 -5.29941 20.0032 64.4691 60.103) #f(11.5088 102.636 -7.16651 112.765 -109.115 13.1269 -65.5666 -78.0758 -7.41119 72.9477 -104.946 -88.7991 69.993 -5.29941 20.0032 64.1682 58.6937) #f(11.5088 100.142 -12.1298 112.122 -109.903 2.77933 -65.4245 -78.0783 -7.41119 72.9477 -104.955 -88.7991 69.993 -5.29941 20.0032 60.1354 56.8041) #f(11.5088 100.494 -12.3333 109.319 -113.229 -1.17475 -65.4245 -78.0783 -7.41594 72.9477 -104.955 -88.7991 69.993 -5.29941 20.0032 61.0708 57.1668) #f(11.5088 100.513 -12.2606 105.92 -114.564 -6.70315 -63.8639 -78.0683 -7.42069 72.9428 -104.955 -88.7991 69.993 -5.29941 20.0032 59.4012 55.9563) #f(11.5088 100.665 -12.0813 99.1854 -114.224 -6.38166 -62.8194 -78.0509 -7.42544 72.9428 -104.964 -88.7908 69.993 -5.29941 20.0032 58.3135 53.9058) #f(11.5088 100.271 -11.6063 96.5761 -115.842 -6.65012 -63.4352 -78.193 -7.42544 72.9428 -104.964 -88.7908 69.993 -5.29941 20.0032 56.1609 54.399) #f(11.5088 100.199 -11.2476 92.6805 -115.286 -10.6771 -62.089 -78.4173 -7.44444 72.9283 -104.964 -88.7908 69.993 -5.29941 20.0032 56.5597 52.1163) #f(11.5088 98.8407 -15.542 93.0113 -115.883 -27.5407 -53.3465 -75.1741 -7.46819 72.9186 -104.973 -88.7908 69.993 -5.29941 20.0032 53.417 47.5959) #f(11.5088 94.138 -20.3065 93.1767 -115.891 -32.688 -49.4776 -71.3101 -7.48244 72.9089 -105.028 -88.7743 69.993 -5.29941 20.0032 47.4559 46.4945) #f(11.5088 85.7539 -20.481 93.8014 -115.692 -45.4816 -38.4216 -60.2542 -7.70095 72.8314 -105.203 -88.7494 69.993 -5.29941 20.0032 40.526 43.9182) #f(11.5088 77.0563 -20.4907 93.7187 -114.473 -64.42 -24.5313 -46.3689 -7.73421 72.7926 -105.276 -88.7494 69.993 -5.29941 20.0032 33.8047 41.0685) #f(11.5088 69.1472 -20.515 93.7279 -110.342 -76.6104 -16.7436 -38.5811 -7.74371 72.7926 -105.285 -88.7494 69.993 -5.29941 20.0032 28.5389 38.8975) #f(11.5088 62.592 -20.2387 96.1535 -105.291 -76.7794 -15.9708 -37.8083 -7.75321 72.7926 -105.304 -88.7494 69.993 -5.29941 20.0032 24.6299 37.9168) #f(11.5088 54.9346 -20.3211 101.712 -97.5434 -76.8026 -15.9633 -37.8009 -7.82446 72.7684 -105.423 -88.7328 69.993 -5.29941 20.0032 21.009 36.9949) #f(11.5088 43.9237 -20.3356 106.48 -83.2433 -82.7088 -15.9683 -37.7959 -7.80071 72.6763 -105.46 -88.7245 69.993 -5.29941 20.0032 16.5595 32.9589) #f(11.5088 34.0005 -20.3696 111.497 -68.8436 -85.9768 -15.9483 -37.7759 -7.82446 72.6763 -105.478 -88.7245 69.993 -5.29941 20.0032 13.3715 29.5216) #f(11.5088 25.4216 -20.3405 120.547 -58.8318 -95.8703 -15.9483 -37.7759 -7.82446 72.6763 -105.488 -88.7245 69.993 -5.29941 20.0032 11.0344 28.9091) #f(11.5088 17.5268 -20.3211 129.045 -49.3178 -104.435 -15.9483 -37.7759 -7.81971 72.6763 -105.506 -88.7245 69.993 -5.29941 20.0032 8.88368 27.6934) #f(11.5088 10.3065 -20.3162 139.887 -42.8562 -114.789 -15.9483 -37.7759 -7.82446 72.6763 -105.506 -88.7245 69.993 -5.29941 20.0032 6.75793 27.4731) #f(11.5088 4.58252 -20.3889 152.308 -36.7927 -128.176 -15.9483 -37.7759 -7.82446 72.6763 -105.515 -88.7245 69.993 -5.29941 20.0032 6.081 26.5202) #f(11.5088 -0.6237 -20.3889 165.575 -30.356 -143.535 -15.9359 -37.7784 -7.82446 72.6763 -105.515 -88.7162 69.993 -5.29941 20.0032 5.56225 24.5291) #f(11.5088 -4.77062 -20.3986 175.425 -28.1496 -155.297 -9.40704 -35.6819 -7.82446 72.6714 -105.524 -88.7162 69.993 -5.29941 20.0032 4.54259 24.8059) #f(11.5088 -6.5852 -20.4035 177.023 -26.5736 -155.785 -6.37819 -34.9016 -7.82446 72.6714 -105.524 -88.7162 69.993 -5.29941 20.0032 3.35129 24.6352))))

    (send *pr2* :torso :waist-z :joint-angle 0)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    ;;(pr2-tuckarm-pose :larm)
    (send *pr2* :larm :inverse-kinematics (send (send *pr2* :copy-worldcoords) :transform dustcds) :rotation-axis t)
    (send *pr2* :look-at-hand :larm)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (send *ri* :move-gripper :larm 0.09)
;;    (send *pr2* :larm :inverse-kinematics (send (send *pr2* :larm :end-coords :copy-worldcoords) :translate #f(0 0 -45)) :rotation-axis t)
    (send *pr2* :larm :elbow-r :joint-angle 110)
    (send *pr2* :look-at-hand :larm)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *ri* :move-gripper :larm 0.0 :effort 100 :wait t)
    (send *ri* :angle-vector-sequence after-avs (make-sequence 'list (length after-avs) :initial-element 300))
    (send *ri* :wait-interpolation)))

;; (defun turn (ri rad)
;;   ;; send *ri* :go-pos x[m] y[m] r[degree] // absolute
;;   (send ri :orient rad :z :world))

;; (defun move (ri x y z rad)
;;   (send ri :translate (float-vector x y z) :world)
;;   (send ri :orient rad :z :world))

;; (defun 73b2-move-to (73b2-spot)
;;   (send *pr2* :move-to 73b2-spot :world)
;;   (send *ri* :move-to (send *pr2* :coords) :frame-id "/eng2/7f/73B2" :retry 3))
