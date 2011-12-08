#-:jsk (jsk)
#-:rbrain (rbrain)

  ;; this file is originally created by
  ;;   (output-domain-template "pr2_action.l" *domain*)
(defun init()
  (load "plan.l")
  (load "pr2_action_function.l")
  (print "start")
  (dotimes (i 5000)
    (x::window-main-one)
    (unix::usleep 1000)))
;; package generate
(let ((ns (string-upcase "pr2_action")))
  (when (and ns (not (find-package ns)))
      (make-package ns)))

;; action definition
(defun PR2_ACTION::move-to-room (spot)
  (let (where)
    (case spot
      ('TABLE-FRONT (setq where (send *room73b2* :spot "/eng2/7f/room73B2-table-side")))
      ('TRASHBOX-FRONT (setq where (send *room73b2* :spot "/eng2/7f/room73B2-front-of-trashbox")))
      ('KITCHEN-INSIDE (setq where (send *room73b2* :spot "/eng2/7f/room73B2-front-of-sink")))
      (t (progn
	   (print "no match move-to-room")
	   (return-from PR2_ACTION::move-to-room t))))
    (move-to-room where "/eng2/7f/73B2")
    t))

(defun PR2_ACTION::pick (obj arm)
  (let (use-arm where)
    (case arm
      ('L-ARM (setq use-arm :larm))
      ('R-ARM (setq use-arm :rarm)))
    (print use-arm)
    (case obj
      ('CUP (setq where (send (send *room73b2* :object "room73b2-sushi-cup2") :handle-handle0)))
      ('EMPTY-BOX (setq where (send (send (send (send *room73b2* :object "room73b2-pockey") :handle-handle0) :copy-worldcoords) :translate #f(0 0 25) :world)))
      ('BOTTLE (setq where (send (send *room73b2* :object "room73b2-pet2") :handle-handle0)))
      (t (progn
	   (print "no match pick")
	   (return-from PR2_ACTION::pick t))))
      (pick where :arm use-arm :use-torso nil)
      t))

(defun PR2_ACTION::release (base arm)
  (let (use-arm where)
    (case arm
      ('L-ARM (setq use-arm :larm))
      ('R-ARM (setq use-arm :rarm)))
    (case base
      ('TRASHBOX (setq where (send (send (send *room73b2* :object "room73b2-trashbox0") :copy-worldcoords) :translate #f(0 0 700))))
      ('KITCHEN-SINK (setq where (send (send (send *room73b2* :object "room73b2-kitchen") :copy-worldcoords) :translate #f(50 400 750))))
      (t (progn
	   (print "no match release")
	   (return-from PR2_ACTION::release t))))
    (release where :arm use-arm :use-torso nil :rotation-axis nil)
    t))

(defun PR2_ACTION::wipe-table4 (base)
  (case base
    ('TABLE (wipe-table4))
    (t print "no match wipe-table4"))
  t)

(init)
