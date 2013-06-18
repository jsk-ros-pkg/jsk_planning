#!/usr/bin/env roseus

(load "package://pddl_planner/src/pddl-result-graph.l")
(load "package://pddl_planner/src/eus-pddl-client.l")

;;
;; problem
;;
(setq *making-tea-problem* (instance pddl-problem :init :name 'making_tea :domain 'okada_ias))

;;使用する変数の宣言
(send *making-tea-problem* :objects
      '((BAR . spot)))

;;初期条件
(send *making-tea-problem* :initial-condition
      '((AT BAR)
        (ON CUP BAR)
        (ON BOTTLE BAR)))

;;終了条件
(send *making-tea-problem* :goal-condition
      '((POURED CUP)
        (ON CUP BAR)
        (ON BOTTLE BAR)))

;;
(setq *cleaning-up-cup-problem* (instance pddl-problem :init :name 'cleaning_up_cup :domain 'okada_ias))

;;使用する変数の宣言
(send *cleaning-up-cup-problem* :objects
      '((BAR . spot)))

;;初期条件
(send *cleaning-up-cup-problem* :initial-condition
      '((AT BAR)
        (ON CUP BAR)
        (ON BOTTLE BAR)))

;;終了条件
(send *cleaning-up-cup-problem* :goal-condition
      '((WASHED CUP)
        (ON CUP SINK)))

;;
;; domain
;;
(setq *domain* (instance pddl-domain :init :name 'okada_ias))
(send *domain* :requirements '(:typing))
(send *domain* :types '(object spot arm))
(send *domain* :constants '(CUP BOTTLE - object LARM RARM - arm SINK - spot))
(send *domain* :predicates '((ON ?obj - object ?spot - spot)
                             (AT ?spot - spot)
                             (HOLD ?obj - object ?arm - arm)
                             (WATERFLOW)
                             (WASHED ?obj - object)
                             (POURED ?obj - object)
                             ))
;;making action
(setq *actions*
      (list
       (instance pddl-action :init
                 :name "hold"
                 :parameters '((?OBJECT object) (?ARM arm) (?SPOT spot))
                 :precondition '((ON ?OBJECT ?SPOT)
                                 (AT ?SPOT))
                 :effect '((HOLD ?OBJECT ?ARM)
                           (NOT (ON ?OBJECT ?SPOT))))
       (instance pddl-action :init
                 :name "place"
                 :parameters '((?OBJECT object) (?ARM arm) (?SPOT spot))
                 :precondition '((HOLD ?OBJECT ?ARM)
                                 (AT ?SPOT))
                 :effect '((NOT (HOLD ?OBJECT ?ARM))
                           (ON ?OBJECT ?SPOT)))
       (instance pddl-action :init
                 :name "move-to"
                 :parameters '((?FROM ?TO spot))
                 :precondition '((AT ?FROM))
                 :effect '((AT ?TO)
                           (NOT (AT ?FROM))))
       (instance pddl-action :init
                 :name "open-tap"
                 :parameters '()
                 :precondition '((AT SINK)
                                 (NOT (WATERFLOW)))
                 :effect '((WATERFLOW)))
       (instance pddl-action :init
                 :name "close-tap"
                 :parameters '()
                 :precondition '((AT SINK)
                                 (WATERFLOW))
                 :effect '((NOT (WATERFLOW))))
       (instance pddl-action :init
                 :name "wash-cup"
                 :parameters '()
                 :precondition '((HOLD CUP LARM)
                                 (AT SINK)
                                 (WATERFLOW)
                                 (NOT (WASHED CUP)))
                 :effect '((WASHED CUP)))
       (instance pddl-action :init
                 :name "pour-tea"
                 :parameters '()
                 :precondition '((HOLD CUP LARM)
                                 (HOLD BOTTLE RARM)
                                 (NOT (POURED CUP)))
                 :effect '((POURED CUP)))))

;;add action to domain
(dolist (act *actions*)
  (send *domain* :add :action act))

;;
;; solve planning
;;
(pprint (setq *result0* (solve-pddl-planning *domain* *making-tea-problem*)))
(pprint (setq *result1* (solve-pddl-planning *domain* *cleaning-up-cup-problem*)))

(setq gr (make-graph-from-pddl-results (list *result0* *result1*) :node-name :pprint))

(send gr :write-to-pdf "okada_ias.pdf")

(when (ros::get-param "~display_graph" "true")
  (piped-fork "gnome-open okada_ias.pdf"))

(when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
  (ros::exit))
