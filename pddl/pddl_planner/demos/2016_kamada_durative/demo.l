#!/usr/bin/env roseus
(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")

;;
;; problem
;;
(setq *problem* (instance pddl-problem :init :name 'demo-problem :domain 'demo-domain))

;;使用する変数の宣言
(send *problem* :objects
      '(
        (fridge . object)
        (cart . object)
        (chair . object)

        (init-pos . position)
        (fridge-front . position)
        (tableclothbox-front . position)
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

;;
;; domain
;;
(setq *domain* (instance pddl-domain :init :name 'demo-domain :durative-action t))
(send *domain* :requirements '(:typing :durative-actions :adl))
(send *domain* :types '(object position area))
(send *domain* :constants '(GEORGIA EMPTY-BOX SUSHI-CUP MUG-CUP BOTTLE - placeable-object
                                    TABLECLOTH - cloth
                                    TABLE-FRONT - position
                                    ))
(send *domain* :predicates '(
                             (closed ?obj - object)
                             (slightly-closed ?obj - object)
                             (connected ?from ?to - position)
                             (at-robot ?robot - robot ?pos - position)
                             (at-object ?object - object ?pos - position)
                             (grasping ?robot - robot ?obj - object)
                             (in ?container - object ?obj - object)
                             (is-blocking ?obj - object)
                             (slightly-blocking ?obj - object)
                             (on ?area - area ?obj - object)
                             (free-arm ?robot)
                             (reachable ?pos - position ?area - area)
                             (tableclothset)
                             ))

;;making action
(setq action-lst
      (list
       (instance pddl-action :init
                 :use-durative-action t
                 :name "pick"
                 :parameters '((?robot - robot ?where - position ?obj - placeable-object))
                 :duration '((= ?duration 3))
                 :precondition '((over all (at-robot ?robot ?where))
                                 (at start (not (grasping ?robot ?obj)))
                                 (at start (at-object ?obj ?where))
                                 (at start (free-arm ?robot))
                                 )
                 :effect '((at end (grasping ?robot ?obj))
                           (at end (not (at-object ?obj ?where)))
                           (at start (not (free-arm ?robot)))
                           ))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "place"
                 :parameters '((?robot - robot ?where - position ?obj - placeable-object))
                 :duration '((= ?duration 3))
                 :precondition '(
                                 (over all (at-robot ?robot ?where))
                                 (at start (grasping ?robot ?obj))
                                 (at start (not (at-object ?obj ?where)))
                                 (at start (not (free-arm ?robot)))
                                 )
                 :effect '(
                           (at start (not (grasping ?robot ?obj)))
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
                           (at start (slightly-closed ?obj))
                           (at end (not (slightly-closed ?obj)))
                           (at end (not (closed ?obj)))
                           ))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "grasp-can"
                 :parameters '((?robot - robot ?pos - position ?fridge - object ?can - placeable-object))
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
                           (at start (not (free-arm ?robot)))
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
                           (at start (slightly-closed ?fridge))
                           (at end (not (slightly-closed ?fridge)))
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
                           (at start (slightly-blocking ?cart))
                           (at end (not (slightly-blocking ?cart)))
                           (at end (not (is-blocking ?cart)))
                           ))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "set-tablecloth"
                 :parameters '((?robot - robot ?pos - position ?tablecloth - cloth))
                 :duration '((= ?duration 18))
                 :precondition '(
                                 (over all (at-robot ?robot ?pos))
                                 (over all (not (at-object GEORGIA TABLE-FRONT)))
                                 (over all (not (at-object EMPTY-BOX TABLE-FRONT)))
                                 (over all (not (at-object SUSHI-CUP TABLE-FRONT)))
                                 (over all (not (at-object MUG-CUP TABLE-FRONT)))
                                 (over all (not (at-object BOTTLE TABLE-FRONT)))
                                 (at start (grasping ?robot ?tablecloth))
                                 (at start (not (free-arm ?robot)))
                                 (at start (not (tableclothset)))
                                 )
                 :effect '(
                           (at start (not (grasping ?robot ?tablecloth)))
                           (at end (at-object ?tablecloth ?pos))
                           (at end (free-arm ?robot))
                           (at end (tableclothset))
                           ))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "get-tablecloth"
                 :parameters '((?robot - robot ?pos - position ?tablecloth - cloth))
                 :duration '((= ?duration 18))
                 :precondition '(
                                 (over all (at-robot ?robot ?pos))
                                 (at start (not (grasping ?robot ?tablecloth)))
                                 (at start (free-arm ?robot))
                                 (at start (at-object ?tablecloth ?pos))
                                 )
                 :effect '(
                           (at end (grasping ?robot ?tablecloth))
                           (at start (not (free-arm ?robot)))
                           (at end (not (at-object ?tablecloth ?pos)))
                           ))
       ))

;;add action to domain
(dolist (ac action-lst)
  (send *domain* :add :action ac)
  )

;;初期条件
(send *problem* :initial-condition
      '(
        (at-robot robot1 init-pos)
        (at-robot robot2 init-pos)
        (free-arm robot1)
        (free-arm robot2)

        (at-object empty-box table-front)
        (at-object sushi-cup table-front)
        (at-object mug-cup table-front)
        (at-object bottle table-front)
        (at-object fridge fridge-front)
        (at-object cart cart-pos)
        (in fridge GEORGIA)
        (is-blocking cart)
        (closed fridge)

        (not (tableclothset))
        (at-object tablecloth tableclothbox-front)
        ))

;;終了条件
(send *problem* :goal-condition
      '(
        (at-object empty-box trash-box-front)
        (at-object sushi-cup kitchen-desk-front)
        (at-object mug-cup kitchen-desk-front)
        (at-object bottle kitchen-desk-front)
        (at-object GEORGIA table-front)
        (closed fridge)
        (not (is-blocking cart))

        ;; (at-object tablecloth table-front)
        ;; (tableclothset)

        ))

;;
;; solve planning
;;

;;  (pprint (setq *result* (solve-pddl-planning *domain* *problem*)))

(defun demo ()
  (setq *result* (solve-pddl-planning *domain* *problem*))
  (setq gr (make-graph-from-pddl-results (list *result*) :node-name :pprint))
  (send gr :write-to-pdf "durativedemo.pdf")
  (when (ros::get-param "~display_graph" "true")
    (piped-fork "gnome-open durativedemo.pdf"))
  (when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
    (ros::exit))
  )

(demo)

