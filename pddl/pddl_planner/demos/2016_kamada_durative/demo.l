#!/usr/bin/env roseus
(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")
(require :tools "package://pddl_planner/demos/2016_kamada_durative/tools.l")

(setq *problem* (instance pddl-problem :init :name 'demo-problem :domain 'demo-domain))
(send *problem* :objects
      '((init-pos . position)
        (kitchentable-front . position)
        (trashbox-front . position)
        (fridge-fron . position)
        (kitchentable . object)
        (trashbox . object)
        (robot1 . robot)
        (robot2 . robot)))

(setq *domain* (instance pddl-domain :init :name 'demo-domain :durative-action t))
(send *domain* :requirements '(:typing :durative-actions :adl))
(send *domain* :types '(object position robot))
(send *domain* :constants '(GEORGIA EMPTY-BOX SUSHI-CUP MUG-CUP BOTTLE TABLECLOTH TABLE FRIDGE CART - object
                                    TABLE-FRONT FRIDGE-FRONT CART-FRONT - position))
(send *domain* :predicates '((at-robot ?robot - robot ?robotpos - position)
                             (reachable-from ?robotpos - position ?obj - object)
                             (grasping ?robot - robot ?obj - object)
                             (placing-possible ?obj) ;; avoiding (on CUP GEORGIA) etc
                             (free-arm ?robot - robot)
                             (on ?underobj - object ?obj - object)
                             (in ?fridge - object ?georgia - object)
                             (closed ?fridge - object)
                             (slightly-closed ?fridge - object)
                             (tableclothset)
                             (cangrasped)
                             (blocking ?cart - object)
                             (slightly-blocking ?cart - object)
                             ))

(setq action-lst
      (list
       (instance pddl-action :init
                 :use-durative-action t
                 :name "pick"
                 ;; ?robot pick ?obj from ?underobj at ?robotpos
                 :parameters '((?robot - robot ?obj - object ?underobj - object ?robotpos - position))
                 :duration '((= ?duration 3))
                 :precondition '((over all (at-robot ?robot ?robotpos))
                                 (over all (reachable-from ?robotpos ?underobj))
                                 (over all (placing-possible ?underobj))
                                 (at start (reachable-from ?robotpos ?obj))
                                 (at start (on ?underobj ?obj))
                                 (at start (free-arm ?robot))
                                 (at start (not (grasping ?robot ?obj)))
                                 )
                 :effect '((at start (not (free-arm ?robot)))
                           (at end (not (reachable-from ?robotpos ?obj)))
                           (at end (not (on ?underobj ?obj)))
                           (at end (grasping ?robot ?obj))
                           ))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "place"
                 ;; ?robot place ?obj on ?underobj at ?robotpos
                 :parameters '((?robot - robot ?obj - object ?underobj - object ?robotpos - position))
                 :duration '((= ?duration 3))
                 :precondition '((over all (at-robot ?robot ?robotpos))
                                 (over all (reachable-from ?robotpos ?underobj))
                                 (over all (placing-possible ?underobj))
                                 (at start (not (reachable-from ?robotpos ?obj)))
                                 (at start (not (on ?underobj ?obj)))
                                 (at start (not (free-arm ?robot)))
                                 (at start (grasping ?robot ?obj))
                                 )
                 :effect '((at start (not (grasping ?robot ?obj)))
                           (at end (reachable-from ?robotpos ?obj))
                           (at end (on ?underobj ?obj))
                           (at end (free-arm ?robot))))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "move"
                 ;; ?robot move from ?from to ?to
                 :parameters '((?robot - robot ?from - position ?to - position))
                 :duration '((= ?duration 20))
                 :precondition '((at start (at-robot ?robot ?from))
                                 )
                 :effect '((at start (not (at-robot ?robot ?from)))
                           (at end (at-robot ?robot ?to))))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "set-tablecloth"
                 ;; ?robot set TABLECLOTH on TABLE at TABLE-FRONT
                 :parameters '((?robot - robot))
                 :duration '((= ?duration 18))
                 :precondition '((at start (not (tableclothset)))
                                 (over all (at-robot ?robot TABLE-FRONT))
                                 (over all (reachable-from TABLE-FRONT TABLE))
                                 (over all (not (on TABLE GEORGIA)))
                                 (over all (not (on TABLE EMPTY-BOX)))
                                 (over all (not (on TABLE SUSHI-CUP)))
                                 (over all (not (on TABLE MUG-CUP)))
                                 (over all (not (on TABLE BOTTLE)))
                                 (at start (free-arm ?robot))
                                 (at start (not (on TABLE TABLECLOTH)))
                                 (at start (not (reachable-from TABLE-FRONT TABLECLOTH)))
                                 )
                 :effect '((at start (not (free-arm ?robot)))
                           (at end (free-arm ?robot))
                           (at end (on TABLE TABLECLOTH))
                           (at end (reachable-from TABLE-FRONT TABLECLOTH))
                           (at end (tableclothset))))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "open-fridge"
                 ;; ?robot open FRIDGE at FRIDGE-FRONT
                 :parameters '((?robot - robot))
                 :duration '((= ?duration 20))
                 :precondition '((over all (at-robot ?robot FRIDGE-FRONT))
                                 (over all (reachable-from FRIDGE-FRONT FRIDGE))
                                 (at start (free-arm ?robot))
                                 (at start (closed FRIDGE))
                                 )
                 :effect '((at start (not (free-arm ?robot)))
                           (at end (free-arm ?robot))
                           (at end (not (closed FRIDGE)))))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "grasp-can"
                 ;; ?robot grasp GEORGIA from FRIDGE at FRIDGE-FRONT
                 :parameters '((?robot - robot))
                 :duration '((= ?duration 20))
                 :precondition '((over all (at-robot ?robot FRIDGE-FRONT))
                                 (over all (reachable-from FRIDGE-FRONT FRIDGE))
                                 (over all (reachable-from FRIDGE-FRONT GEORGIA))
                                 (over all (not (closed FRIDGE)))
                                 (at start (in FRIDGE GEORGIA))
                                 (at start (not (cangrasped)))
                                 (at start (free-arm ?robot))
                                 (at start (not (grasping ?robot GEORGIA)))
                                 )
                 :effect '((at start (not (free-arm ?robot)))
                           (at end (not (closed FRIDGE)))
                           (at end (not (in FRIDGE GEORGIA)))
                           (at end (cangrasped))
                           (at end (not (reachable-from FRIDGE-FRONT GEORGIA)))
                           (at end (grasping ?robot GEORGIA))))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "close-fridge"
                 ;; ?robot close FRIDGE at FRIDGE-FRONT
                 :parameters '((?robot - robot))
                 :duration '((= ?duration 20))
                 :precondition '((over all (at-robot ?robot FRIDGE-FRONT))
                                 (over all (reachable-from FRIDGE-FRONT FRIDGE))
                                 (at start (cangrasped))
                                 (at start (not (closed FRIDGE)))
                                 )
                 :effect '((at start (slightly-closed FRIDGE))
                           (at end (not (slightly-closed FRIDGE)))
                           (at end (closed FRIDGE))))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "remove-cart"
                 ;; ?robot remove CART at CART-FRONT
                 :parameters '((?robot - robot))
                 :duration '((= ?duration 20))
                 :precondition '((over all (at-robot ?robot CART-FRONT))
                                 (over all (reachable-from CART-FRONT CART))
                                 (at start (blocking CART))
                                 )
                 :effect '((at start (slightly-blocking CART))
                           (at end (not (slightly-blocking CART)))
                           (at end (not (blocking CART)))))
       ))
(dolist (ac action-lst)
  (send *domain* :add :action ac))

(send *problem* :initial-condition
      '((at-robot robot1 init-pos)
        (at-robot robot2 init-pos)
        (reachable-from TABLE-FRONT TABLE)
        (reachable-from TABLE-FRONT EMPTY-BOX)
        (reachable-from TABLE-FRONT SUSHI-CUP)
        (reachable-from TABLE-FRONT MUG-CUP)
        (reachable-from TABLE-FRONT BOTTLE)
        (reachable-from FRIDGE-FRONT FRIDGE)
        (reachable-from FRIDGE-FRONT GEORGIA)
        (reachable-from CART-FRONT CART)
        (reachable-from kitchentable-front kitchentable)
        (reachable-from trashbox-front trashbox)
        (not (reachable-from TABLE-FRONT TABLECLOTH))
        (placing-possible TABLE)
        (placing-possible TABLECLOTH)
        (placing-possible kitchentable)
        (placing-possible trashbox)
        (free-arm robot1)
        (free-arm robot2)
        (not (busy robot1))
        (not (busy robot2))
        (on TABLE EMPTY-BOX)
        (on TABLE SUSHI-CUP)
        (on TABLE MUG-CUP)
        (on TABLE BOTTLE)
        (closed FRIDGE)
        (in FRIDGE GEORGIA)
        (blocking CART)
        (not (tableclothset))))

(send *problem* :goal-condition
      '(
        (reachable-from TABLE-FRONT TABLE)
        (reachable-from TABLE-FRONT GEORGIA)
        (reachable-from kitchentable-front SUSHI-CUP)
        (reachable-from kitchentable-front MUG-CUP)
        (reachable-from kitchentable-front BOTTLE)
        (reachable-from kitchentable-front kitchentable)
        (reachable-from FRIDGE-FRONT FRIDGE)
        (reachable-from TABLE-FRONT TABLECLOTH)
        (reachable-from trashbox-front EMPTY-BOX)
        (on kitchentable SUSHI-CUP)
        (on kitchentable MUG-CUP)
        (on kitchentable BOTTLE)
        (on trashbox EMPTY-BOX)
        (on TABLECLOTH GEORGIA)
        (on TABLE TABLECLOTH)
        (closed FRIDGE)
        (not (blocking CART))
        (tableclothset)
        ))

(warn "~%")
(warn ";(demo)~%")
;(demo)
