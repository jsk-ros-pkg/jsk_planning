#!/usr/bin/env roseus
;; simplified version of pddl_planner/demos/hanoi/solve-hanoi.l

(load "package://pddl_planner/src/pddl-result-graph.l")
(load "package://pddl_planner/src/eus-pddl-client.l")

(setq *action-client*
      (instance ros::simple-action-client :init
                "pddl_planner" pddl_msgs::PDDLPlannerAction))
;;
;; problem
;;
;;hanoi tower
(setq *problem* (instance pddl-problem :init :name 'hanoi :domain 'manip))

;;使用する変数の宣言
(send *problem* :objects
      '((disk0 . object)
        (disk1 . object)
        (disk2 . object)
        (pole0 . object)
        (pole1 . object)
        (pole2 . object))             ; 簡単のため使う型は一つのみに
      )                            ; disk0,1,2 pole0,1,2すべてobject型

;;初期条件
;; disk0 disk1 disk2
;; pole0 pole1 pole2
(send *problem* :initial-condition
      '((on disk0 pole0)
        (on disk1 pole1)
        (on disk2 pole2)
        (can-be-on disk0 disk1)
        (can-be-on disk1 disk2)
        (clear disk0)
        (clear disk1)
        (clear disk2)))

;;初期条件の追加
(dolist (dd '(disk0 disk1 disk2))
  (dolist (pp '(pole0 pole1 pole2))
    (send *problem* :add :initial-condition
          (list 'can-be-on dd pp))))

;;終了条件
;;
;;             disk0
;;             disk1
;;             disk2
;; pole0 pole1 pole2
(send *problem* :goal-condition
      '((on disk0 disk1)
        (on disk1 disk2)
        (on disk2 pole2)
        (clear pole0)
        (clear pole1)
        (clear disk0)
        ))

;;
;; domain
;;
(setq *domain* (instance pddl-domain :init :name 'manip))
(send *domain* :requirements '(:typing))
(send *domain* :types '(object))
(send *domain* :predicates '((on ?obj0 ?obj1 - object)
                             (clear ?obj - object)
                             (can-be-on ?obj0 ?obj1 - object)))
;;making action
(setq *move* (instance pddl-action :init
                       :name "move"
                       :parameters '((?obj ?from ?to object))
                       :precondition '((clear ?obj)
                                       (clear ?to)
                                       (on ?obj ?from)
                                       (can-be-on ?obj ?to))
                       :effect '((on ?obj ?to)
                                 (not (on ?obj ?from))
                                 (not (clear ?to))
                                 (clear ?from))))
;;add action to domain
(send *domain* :add :action *move*)


;;
;; solve planning
;;
(pprint (setq *result* (solve-pddl-planning *domain* *problem*)))
(setq *gr* (make-graph-from-pddl-results (list *result*) :node-name :pprint))

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(deftest test-graph-solve ()
  (let (solver path result-path solved-path)
    ;; solve graph search
    (send *gr* :start-state (elt (send *gr* :nodes) 2))
    (send *gr* :goal-state  (elt (send *gr* :nodes) 0))
    (setq solver (instance breadth-first-graph-search-solver :init))
    (setq path (send solver :solve *gr* :verbose nil))

    (setq result-path (cdr (assoc :plan *result*)))
    (setq solved-path (mapcar #'read-from-string (cdr (send-all (send-all path :action) :name))))
    (format *error-output* "result-path ~A~%" result-path)
    (format *error-output* "solved-path ~A~%" solved-path)
    (assert (equal solved-path result-path))
    ))

(run-all-tests)
(exit)
