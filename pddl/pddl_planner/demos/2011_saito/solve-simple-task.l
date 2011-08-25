#!/usr/bin/env roseus

(load "package://pddl_planner/src/pddl-result-graph.l")
(load "package://pddl_planner/src/eus-pddl-client.l")

;;
;; domain
;;
(setq *domain* (instance pddl-domain :init :name 'pr2_action))
(send *domain* :requirements '(:typing))
(send *domain* :types '(object spot))
(send *domain* :predicates
      '(
        (AT ?SPOT - spot) ;; the spot which robot is in
        (OPEN ?OBJ - object) ;; the object is enable (door -> close)
        (CLEAN ?OBJ - object)
        (REACHABLE ?OBJ - object ?SPOT - spot)
        (SEPARATE ?OBJ - object ?FROM ?TO - spot)
        ))
;;making action
(setq *actlist*
      (list
       (instance pddl-action :init
                 :name "move"
                 :parameters '((?FROM ?TO spot))
                 :precondition '((AT ?FROM)
                                 (NOT (= ?FROM ?TO))
                                 (forall (?obs - object)
                                         (NOT (OR (SEPARATE ?obs ?FROM ?TO)
                                                  (SEPARATE ?obs ?TO ?FROM)))))
                 :effect '((AT ?TO)
                           (NOT (AT ?FROM))))
       (instance pddl-action :init
                 :name "move_f"
                 :parameters '((?FROM ?TO spot))
                 :precondition '((AT ?FROM)
                                 (NOT (= ?FROM ?TO))
                                 (forall (?obs - object)
                                         (NOT (OR (SEPARATE ?obs ?FROM ?TO)
                                                  (SEPARATE ?obs ?TO ?FROM)))))
                 :effect '((AT ?FROM)))

       (instance pddl-action :init
                 :name "knock"
                 :parameters '((?OBJ object))
                 :precondition '((exists (?spot - spot)
                                         (AND (REACHABLE ?OBJ ?spot)
                                              (AT ?spot)))
                                 (NOT (OPEN ?OBJ)))
                 :effect '((OPEN ?OBJ)
                           (forall (?S1 ?S2 - spot)
                                   (NOT (SEPARATE ?OBJ ?S1 ?S2)))
                           ))
       (instance pddl-action :init
                 :name "knock_f"
                 :parameters '((?OBJ object))
                 :precondition '((exists (?spot - spot)
                                         (AND (REACHABLE ?OBJ ?spot)
                                              (AT ?spot)))
                                 (NOT (OPEN ?OBJ)))
                 :effect '((NOT (OPEN ?OBJ))))

       (instance pddl-action :init
                 :name "wipe"
                 :parameters '((?OBJ object))
                 :precondition '((exists (?spot - spot)
                                         (AND (REACHABLE ?OBJ ?spot)
                                              (AT ?spot)))
                                 (NOT (CLEAN ?OBJ)))
                 :effect '((CLEAN ?OBJ)))
       (instance pddl-action :init
                 :name "wipe_f"
                 :parameters '((?OBJ object))
                 :precondition '((exists (?spot - spot)
                                         (AND (REACHABLE ?OBJ ?spot)
                                              (AT ?spot)))
                                 (NOT (CLEAN ?OBJ)))
                 :effect '((NOT (CLEAN ?OBJ))))
       ))

;;add action to domain
(dolist (ac *actlist*)
  (send *domain* :add :action ac))

;;
;; problem
;;
;; mikita knock
(setq *problem*
      (instance pddl-problem :init :name 'mikita-knock :domain 'pr2_action))

;; one object and three spots
(send *problem* :objects
      '((DOOR . object)
        (DESK . object)
        (START-SPOT . spot)
        (ROOM-FRONT . spot)
        (ROOM-INSIDE . spot))
      )

;; initial condition
(send *problem* :initial-condition
      '(;; from sensor-data or heuristics
        (AT ROOM-FRONT)
        ;; generate from knowledge-base/manip-planner
        (REACHABLE DOOR ROOM-FRONT)
        (REACHABLE DESK ROOM-INSIDE)
        ;; generate fron jsk_map
        (SEPARATE DOOR ROOM-INSIDE START-SPOT)
        (SEPARATE DOOR ROOM-INSIDE ROOM-FRONT)
        ))

;;
;; solve planning
;;

;; we want to enter a room
(send *problem* :goal-condition
      '((CLEAN DESK)))
(pprint (setq *result1* (solve-pddl-planning *domain* *problem* :debug nil)))

;(setq results (list *result1* *result2* *result3*))
(setq results (list *result1*))

;; make graph from PDDL answer
(setq *graph* (make-graph-from-pddl-results results :node-name :pprint))
;; swap nodes ( check_open <-> check_open_f ) for making task flow

(dolist (res results)
  (setq *graph* (add-failed-nodes res (list 'knock 'wipe 'move) :graph *graph*)))

(send *graph* :name (send *domain* :name))

(send *graph* :write-to-pdf "knock_demo.pdf")
(when (ros::get-param "~display_graph")
  (piped-fork "gnome-open knock_demo.pdf"))

;; action definition
;; domain -> package
;; this file is created by (output-domain-template "pr2_action.l" *domain*)
(let ((ns (string-upcase (send *domain* :name))))
  (when (and ns (not (find-package ns)))
      (make-package ns)))
(defun PR2_ACTION::move (from to)
  (format t ";; move action [~a ~a]~%" from to) t)
(defun PR2_ACTION::knock (obj)
  (format t ";; knock action [~a]~%" obj) t)
(defun PR2_ACTION::wipe (obj)
  (format t ";; wiping [~a]       ~%" obj) t)

;; convert to smach
(load "package://roseus_smach/src/state-machine-ros.l")
(load "package://roseus_smach/src/state-machine-utils.l")
(load "package://roseus_smach/src/pddl2smach.l")

;; global data is not used (nil)
(exec-smach-with-spin (convert-smach *graph*) nil :hz 1.0)

(when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
  (ros::exit))
