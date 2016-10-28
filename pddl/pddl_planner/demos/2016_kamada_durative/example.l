#!/usr/bin/env roseus
(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")
(require :tools "package://pddl_planner/demos/2016_kamada_durative/tools.l")

(setq *problem* (instance pddl-problem :init :name 'example-problem :domain 'example-domain))
(send *problem* :objects
      '((robot1 . robot)
        (robot2 . robot)))

(setq *domain* (instance pddl-domain :init :name 'example-domain :durative-action t))
(send *domain* :requirements '(:typing :durative-actions :adl))
(send *domain* :types '(object robot position))
(send *domain* :constants '(TABLECLOTH DISH CUP TABLE - object))

(send *domain* :predicates '(
                             (on ?underobj - object ?obj - object)
                             (at-robot ?robot - robot ?obj - object)
                             (working ?robot -robot)
                             ))

(setq action-lst
      (list
       (instance pddl-action :init
                 :use-durative-action t
                 :name "pick"
                 ;; ?robot pick ?obj from ?underobj
                 :parameters '((?robot - robot ?obj - object ?underobj - object))
                 :duration '((= ?duration 10))
                 :precondition '(
                                 (at start (not (working ?robot)))
                                 (over all (at-robot ?robot TABLE))
                                 (over all (not (on ?obj CUP)))
                                 (over all (not (on ?obj DISH)))
                                 (at start (on ?underobj ?obj))
                                 )
                 :effect '(
                           (at start (working ?robot))
                           (at end (not (working ?robot)))
                           (at end (not (on ?underobj ?obj)))
                           ))

       (instance pddl-action :init
                 :use-durative-action t
                 :name "movetoTABLE"
                 ;; ?robot move to TABLE
                 :parameters '((?robot - robot))
                 :duration '((= ?duration 1))
                 :precondition '(
                                 (at start (not (working ?robot)))
                                 (at start (not (at-robot ?robot TABLE)))
                                 )
                 :effect '(
                           (at start (working ?robot))
                           (at end (not (working ?robot)))
                           (at end (at-robot ?robot TABLE))
                           ))
       ))

(dolist (ac action-lst)
  (send *domain* :add :action ac))

(send *problem* :initial-condition
      '(
        (not (at-robot robot1 TABLE))
        (not (at-robot robot2 TABLE))
        (on TABLECLOTH DISH)
        (on TABLECLOTH CUP)
        (on TABLE TABLECLOTH)
        ))

(send *problem* :goal-condition
      '(
        (not (on TABLECLOTH DISH))
        (not (on TABLECLOTH CUP))
        (not (on TABLE TABLECLOTH))
        ))

(warn "~%")
(warn ";(demo)~%")
;(demo)
