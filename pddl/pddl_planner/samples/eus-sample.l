#!/usr/bin/env roseus
;; mostly same to demo-eus.l in pddl package under jsk subversion trunk
(load "package://pddl_planner/src/eus-pddl-client.l")

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
(send *problem* :initial-condition
      '((on disk0 disk1)
        (on disk1 disk2)
        (on disk2 pole0)
        (can-be-on disk0 disk1)
        (can-be-on disk1 disk2)
        (clear pole1)
        (clear pole2)))

;;初期条件の追加
(send *problem* :add :initial-condition
      '(clear disk0))
(dolist (dd '(disk0 disk1 disk2))
  (dolist (pp '(pole0 pole1 pole2))
    (send *problem* :add :initial-condition
          (list 'can-be-on dd pp))))
;;終了条件
(send *problem* :goal-condition
      '((on disk0 disk1)
        (on disk1 disk2)
        (on disk2 pole2)
        (clear pole0)
        (clear pole1)
        (clear disk0)))

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
(setq *result* (solve-pddl-planning *domain* *problem*))
(pprint (cdr (assoc :plan *result*)))
(print *result*)
(ros::exit)
