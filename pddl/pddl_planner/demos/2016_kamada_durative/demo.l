#!/usr/bin/env roseus
(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")

(setq *problem* (instance pddl-problem :init :name 'demo-problem :domain 'demo-domain))

;;使用する変数の宣言
(send *problem* :objects
      '(
        (fridge . object)
        (georgia . object)
        (cart . object)
        (empty-box . object)
        (sushi-cup . object)
        (mug-cup . object)
        (chair . object)
        (bottle . object)
        (tablecloth . object)
        (init-pos . position)
        (fridge-front . position)
        (table-front . position)
        (cart-pos . position)
        (kitchen-desk-front . position)
        (trash-box-front . position)
        (table . area)
        (kitchen-desk . area)
        (trash-box . area)
        (robot1 . robot)
        (robot2 . robot)
        )
      )

;;初期条件
(send *problem* :initial-condition
      '(
        (at-robot robot1 init-pos)
        (at-robot robot2 init-pos)
        (at-object empty-box table-front)
        (at-object sushi-cup table-front)
        (at-object mug-cup table-front)
        (at-object bottle table-front)
        (at-object fridge fridge-front)
        (at-object cart cart-pos)
        (free-arm robot1)
        (free-arm robot2)
        (in fridge georgia)
        (is-blocking cart)
        (closed fridge)
        ;; setting handling-possible to avoid grasping fridge
        (not (handling-possible fridge))
        (handling-possible georgia)
        (handling-possible cart)
        (handling-possible empty-box)
        (handling-possible sushi-cup)
        (handling-possible mug-cup)
        (handling-possible chair)
        (handling-possible bottle)
        ))

;;終了条件
(send *problem* :goal-condition
      '(
        (at-object empty-box trash-box-front)
        (at-object sushi-cup kitchen-desk-front)
        (at-object mug-cup kitchen-desk-front)
        (at-object bottle kitchen-desk-front)
        (at-object georgia table-front)
        (closed fridge)
        (not (is-blocking cart))
        ))

;;
;; domain
;;
(setq *domain* (instance pddl-domain :init :name 'demo-domain :use-durative-action t))
(send *domain* :requirements '(:typing :durative-actions :adl))
(send *domain* :types '(object))
(send *domain* :predicates '(
                             (closed ?obj - object)
                             (connected ?from ?to - position)
                             (done1 ?obj - object)
                             (done2 ?obj - object)
                             (at-robot ?robot - robot ?pos - position)
                             (at-object ?object - object ?pos - position)
                             (grasping ?robot - robot ?obj - object)
                             (in ?container - object ?obj - object)
                             (is-blocking ?obj - object)
                             (on ?area - area ?obj - object)
                             (free-arm ?robot)
                             (reachable ?pos - position ?area - area)
                             (handling-possible ?obj - object)
                             ))

;;making action
(setq action-lst
      (list
       (instance pddl-action :init
                 :use-durative-action t
                 :name "pick"
                 :parameters '((?robot - robot ?where - position ?obj - object))
                 :duration '((= ?duration 3))
                 :precondition '((over all (at-robot ?robot ?where))
                                 (over all (handling-possible ?obj))
                                 (at start (not (grasping ?robot ?obj)))
                                 (at start (at-object ?obj ?where))
                                 (at start (free-arm ?robot))
                                 )
                 :effect '((at end (grasping ?robot ?obj))
                           (at end (not (at-object ?obj ?where)))
                           (at end (not (free-arm ?robot)))
                           ))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "place"
                 :parameters '((?robot - robot ?where - position ?obj - object))
                 :duration '((= ?duration 3))
                 :precondition '(
                                 (over all (at-robot ?robot ?where))
                                 (at start (grasping ?robot ?obj))
                                 (at start (not (at-object ?obj ?where)))
                                 (at start (not (free-arm ?robot)))
                                 )
                 :effect '(
                           (at end (not (grasping ?robot ?obj)))
                           (at end (at-object ?obj ?where))
                           (at end (free-arm ?robot))
                           ))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "move"
                 :parameters '((?robotA - robot ?from - position ?to - position))
                 :duration '((= ?duration 20))
                 :precondition '(
                                 (at start (at-robot ?robotA ?from))
                                 )
                 :effect '(
                           (at start (not (at-robot ?robotA ?from)))
                           (at end (at-robot ?robotA ?to))
                           ))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "open-fridge"
                 :parameters '((?robot - robot ?pos - position ?obj - object))
                 :duration '((= ?duration 5))
                 :precondition '(
                                 (over all (at-robot ?robot ?pos))
                                 (over all (at-object ?obj ?pos))
                                 (at start (closed ?obj))
                                 )
                 :effect '(
                           (at end (not (closed ?obj)))
                           ))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "grasp-can"
                 :parameters '((?robot - robot ?pos - position ?fridge - object ?can - object))
                 :duration '((= ?duration 5))
                 :precondition '(
                                 (over all (at-robot ?robot ?pos))
                                 (over all (at-object ?fridge ?pos))
                                 (over all (not (closed ?fridge)))
                                 (over all (in ?fridge ?can))
                                 (at start (not (grasping ?robot ?can)))
                                 (at start (free-arm ?robot))
                                 )
                 :effect '(
                           (at end (grasping ?robot ?can))
                           (at end (not (in ?fridge ?can)))
                           (at end (not (free-arm ?robot)))
                           ))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "close-fridge"
                 :parameters '((?robot - robot ?pos - position ?fridge - object))
                 :duration '((= ?duration 4))
                 :precondition '(
                                 (over all (at-robot ?robot ?pos))
                                 (over all (at-object ?fridge ?pos))
                                 (at start (not (closed ?fridge)))
                                 )
                 :effect '(
                           (at end (closed ?fridge))
                           ))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "remove-cart"
                 :parameters '((?robot - robot ?pos - position ?cart - object))
                 :duration '((= ?duration 13))
                 :precondition '(
                                 (over all (at-robot ?robot ?pos))
                                 (over all (at-object ?cart ?pos))
                                 (at start (is-blocking ?cart))
                                 )
                 :effect '(
                           (at end (not (is-blocking ?cart)))
                           ))
       ))

;;add action to domain
(dolist (ac action-lst)
  (send *domain* :add :action ac)
  )

;;
;; solve planning
;;

(solve-pddl-planning *domain* *problem*)
