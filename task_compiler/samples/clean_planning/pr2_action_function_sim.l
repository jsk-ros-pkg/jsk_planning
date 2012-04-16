(defun move-to-room (goal room &optional (retry 3))
  (pr2-tuckarm-pose)
  (send *pr2* :move-to goal :world)
  (send *ri* :move-to (send *pr2* :coords) :frame-id room :retry retry)
  t)

(defun pick (obj &key (arm :larm) (use-torso nil))
  (ros::spin-once)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :head :look-at (send obj :worldpos) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
;;  (send *ri* :publish-joint-state)
  (unix::sleep 1)
;;  (send *ri* :move-end-coords-plan (send *pr2* :transformation (send (send (send obj :handle-handle0) :copy-worldcoords) :translate #f(-100 0 0) :local)) :move-arm arm :use-torso use-torso)
  (send *pr2* :inverse-kinematics (send *pr2* :transformation (send (send (send obj :handle-handle0) :copy-worldcoords) :translate #f(-100 0 0) :local)) :move-arm arm :use-torso use-torso)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *ri* :move-gripper arm 0.08 :wait t)
  (ros::spin-once)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :inverse-kinematics (send obj :handle-handle0) :move-arm arm :use-torso use-torso :rotation-axis t)
  (send *pr2* arm :end-coords :assoc obj)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
;;  (send *ri* :start-grasp arm)
  (send *ri* :move-gripper arm 0 :wait t)
  (send *ri* :robot arm :end-coords :assoc obj)
  (pr2-tuckarm-pose)
  t)

(defun release (coords &rest args &key (arm :larm) (use-torso nil) &allow-other-keys)
  (let ((obj (car (send *ri* :robot arm :end-coords :descendants))))
    (ros::spin-once)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send *pr2* :head :look-at (send coords :worldpos) :world)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (send* *pr2* :inverse-kinematics coords :move-arm arm args)
;;    (send *ri* :publish-joint-state)
;;    (send *ri* :move-end-coords-plan (send *pr2* :transformation (send *pr2* arm :end-coords)) :move-arm arm :use-torso use-torso)
    (send *pr2* :inverse-kinematics (send *pr2* :transformation (send *pr2* arm :end-coords)) :move-arm arm :use-torso use-torso)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (send *ri* :move-gripper arm 0.08 :wait nil)
    (send *ri* :robot arm :end-coords :dissoc obj)
    (dotimes (i 30)
      (send obj :translate (float-vector 0 0 (- 0 i)) :world)
      (send *ri* :viewer :draw-objects))
    (unix::sleep 1)
    (send *ri* :move-gripper arm 0.005 :wait nil)
    (pr2-tuckarm-pose)
    t))

(defun move-chair (chair x y &optional (d 0) &key (direct :x))
  (let ((cds-list '(nil nil)) (rc-chair-pos nil))
    (send *pr2* :head :look-at (vector-mean (send-all (send chair :handle) :worldpos)) :world)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (hold-chair4 chair)
    (send *pr2* :larm :end-coords :assoc chair)
    (send *pr2* :translate (float-vector (* x 1000) (* y 1000) 0))
    (send *pr2* :rotate (deg2rad d) :z)
    (if (eq direct :x)
	(progn
	  (send *ri* :go-pos-unsafe x 0 0)
	  (send *ri* :go-pos-unsafe 0 y d))
      (progn
	(send *ri* :go-pos-unsafe 0 y d)
	(send *ri* :go-pos-unsafe x 0 0)))
    (send *pr2* :larm :end-coords :dissoc chair)
    (release-chair4 chair)
    (send (send *room73b2* :spot "/eng2/7f/room73B2-chair-back") :move-to (send *pr2* :copy-worldcoords) :world)
    t))

(defun hold-chair3 (chair)
  (send *pr2* :larm :angle-vector #f(25 0 0 -130 0 0 0))
  (send *pr2* :rarm :angle-vector #f(-25 0 0 -130 0 0 0))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *pr2* :reset-pose)
  (send *pr2* :head :look-at (vector-mean (send-all (send chair :handle) :worldpos)) :world)
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
  (send *pr2* :inverse-kinematics (send (send *pr2* :larm :end-coords :copy-worldcoords) :translate #f(-300 0 0) *pr2*) :move-arm :larm :use-torso nil :rotation-axis :y)
  (send *pr2* :inverse-kinematics (send (send *pr2* :rarm :end-coords :copy-worldcoords) :translate #f(-300 0 0) *pr2*) :move-arm :rarm :use-torso nil :rotation-axis :y)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *pr2* :rarm :angle-vector #f(-14.3227 16.0059 -85.3758 -102.121 -35.757 -32.3901 -173.201))
  (send *pr2* :larm :angle-vector #f(8.64394 19.7097 109.218 -85.0018 43.0966 -7.98218 163.739))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation))

(defun release-chair4 (chair)
  (send *pr2* :translate #f(300 0 0))
  (send *ri* :go-pos-unsafe 0.3 0)
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
  (send *pr2* :translate #f(-300 0 0))
  (send *ri* :go-pos-unsafe -0.3 0)
  )

(defun wipe-table4 (&rest args)
  (let (avs model check-avs (wipe-rot #2f((0.965926 5.682722e-21 0.258817) (-8.512166e-16 1.0 3.176784e-15) (-0.258817 -3.288848e-15 0.965926))))
    (initial-wipe-motion :height 650)
    (setq model (make-cube 1000 1000 20))
    (send model :set-color #f(1 1 1 0))
    (send model :move-to (send *pr2* :worldcoords) :world)
    (send model :locate (float-vector 700 0 (- (elt (send *pr2* :larm :end-coords :worldpos) 2) 30)))
    (setq avs (wipe-motion3 :offset #f(-200 0 0) :width 700 :init t :model model :arm :larm))
    (dotimes (i 10)
      (setq check-avs (apply #'wipe-motion3 :offset #f(50 0 0) :rot wipe-rot :width 700 :model model :arm :larm args))
      (unless check-avs
	(return nil))
      (setq avs (append avs check-avs)))
    (send *ri* :angle-vector-sequence avs (make-sequence 'list (length avs) :initial-element 300))
    (send *ri* :wait-interpolation)

    (send *pr2* :reset-manip-pose)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (pr2-tuckarm-pose :larm)
    (ros::spin-once)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send *pr2* :torso :waist-z :joint-angle 0)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (release-duster)
    t))

(defun initial-wipe-motion (&key (height 700) (base-tm 2000))
    (ros::spin-once)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (catch-duster2)

    (ros::spin-once)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))

    (send *pr2* :torso :waist-z :joint-angle (/ height 5))

    (send *pr2* :larm :angle-vector #f(0.0 -50.0 180.0 -60.0 180.0 -15.0 0.0))
    (send *ri* :angle-vector (send *pr2* :angle-vector) (* 3 base-tm))

    (send *ri* :wait-interpolation)

    (send *pr2* :larm :inverse-kinematics
	  (send (send *pr2* :larm :end-coords :copy-worldcoords) :translate
		(float-vector 0 0
			      (- height (elt (send *pr2* :larm :end-coords :worldpos) 2))))
	  :rotation-axis t :revert-if-fail nil)

    (send *pr2* :look-at-hand :larm)
    (send *ri* :angle-vector (send *pr2* :angle-vector) (* 3 base-tm))
    (send *ri* :wait-interpolation)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))


    (ros::spin-once)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send *pr2* :look-at-hand :larm)

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

(defun catch-duster2()
  (let ((dustcds (make-coords :pos #f(206.134 232.7 337.956) :rpy #f(-1.753 0.049 -0.015)))
	(after-avs '(
		     #f(11.5 96.6532 74.2725 97.297 -78.8943 110.0 -102.139 -77.2674 -5.0 74.2725 -105.0 -90.0 70.0 -5.72958 20.0 46.2766 74.2702)
		       #f(11.5088 98.8407 -15.542 93.0113 -115.883 -27.5407 -53.3465 -75.1741 -7.46819 72.9186 -104.973 -88.7908 69.993 -5.29941 20.0032 53.417 47.5959)
		       #f(11.5088 54.9346 -20.3211 101.712 -97.5434 -76.8026 -15.9633 -37.8009 -7.82446 72.7684 -105.423 -88.7328 69.993 -5.29941 20.0032 21.009 36.9949)
		       #f(11.5088 0.0 -20.2598 180.0 -60.0 180.0 -15.0 0.0 -5.0 74.2725 -105.0 -90.0 70.0 -5.72958 20.0 11.7636 38.9644))))
    (send *pr2* :torso :waist-z :joint-angle 0)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (send *pr2* :larm :inverse-kinematics (send (send *pr2* :copy-worldcoords) :transform dustcds) :rotation-axis t)
    (send *pr2* :look-at-hand :larm)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (send *ri* :move-gripper :larm 0.09)
    (send *pr2* :larm :elbow-r :joint-angle 110)
    (send *pr2* :look-at-hand :larm)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *ri* :move-gripper :larm 0.0 :effort 100 :wait t)
    (send *ri* :angle-vector-sequence after-avs (make-sequence 'list (length after-avs) :initial-element 1500))
    (send *ri* :wait-interpolation)))

(defun release-duster()
  (let ((dustcds (make-coords :pos #f(226.134 232.7 337.956) :rpy #f(-1.753 0.049 -0.015)))
        (ltuckarm #f(11.5 0.0 65.0 70.0 -121.542 50.0 -114.592 160.0 -5.0 74.2725 -105.0 -90.0 70.0 -5.72958 20.0 0.0 0.0)))
    (send *pr2* :torso :waist-z :joint-angle 0)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (send *pr2* :larm :inverse-kinematics (send (send *pr2* :copy-worldcoords) :transform dustcds) :rotation-axis t)
    (send *pr2* :look-at-hand :larm)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (send *ri* :move-gripper :larm 0.09 :wait t)
    (send *pr2* :angle-vector ltuckarm)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (send *ri* :move-gripper :larm 0.005 :wait nil)))