;; this file is originally created by
;;   (output-domain-template "pr2_action.l" *domain*)
(defun init()
  (load "package://pr2eus/pr2-interface.l")
  (load "package://pr2eus_openrave/pr2eus-openrave.l")
  (load "package://euslisp/jskeus/eus/models/room73b2-scene.l")
  (load "pr2_action_function_sim.l")
  (when (not (boundp '*ri*))
    (ros::roseus "plan")
    (pr2)
    (room73b2)
    (setq *ri* (instance pr2-interface :init)))
  (send *ri* :objects (send *room73b2* :objects))
  (send *ri* :viewer :look-all)
  (print "start")
  (dotimes (i 5000)
    (x::window-main-one)
    (unix::usleep 1000)))
;; package generate
(let ((ns (string-upcase "pr2_action")))
  (when (and ns (not (find-package ns)))
      (make-package ns)))

(defun symbol2eus(sym type)
  (case type
    (:spot (send *room73b2* :spot (format nil "/eng2/7f/room73B2-~A" (string-downcase sym))))
    (:object (send *room73b2* :object (format nil "room73b2-~A" (string-downcase sym))))
    (:arm
     (case sym
       ('L-ARM :larm)
       ('R-ARM :rarm)))
    (:base
     (case sym
       ('KITCHEN-SINK (send (send (send *room73b2* :object "room73b2-kitchen") :copy-worldcoords) :translate #f(180 400 900)))
       ('TRASHBOX (send (send (send *room73b2* :object "room73b2-trashbox0") :copy-worldcoords) :translate #f(0 0 700)))))
    (t nil)))

;; action definition
(defun PR2_ACTION::move-to-room (spot)
  (move-to-room (symbol2eus spot :spot) "/eng2/7f/73B2"))

(defun PR2_ACTION::move-chair (object spot)
  (let ((obj (symbol2eus object :object)))
    (case spot
      ('TMP-CHAIR-SPOT (move-chair obj 0 1.0 0 :direct :x))
      ('TABLE-FRONT (move-chair obj 0 -1.0 0 :direct :y)))))

(defun PR2_ACTION::pick (object arm)
  (pick (symbol2eus object :object) :arm (symbol2eus arm :arm) :use-torso t))

(defun PR2_ACTION::release (base arm)
  (release (symbol2eus base :base) :arm (symbol2eus arm :arm) :use-torso t :rotation-axis nil))

(defun PR2_ACTION::wipe-table4 (base)
  (wipe-table4))

(init)