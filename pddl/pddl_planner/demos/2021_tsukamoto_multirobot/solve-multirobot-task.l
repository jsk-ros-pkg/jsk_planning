#!/usr/bin/env roseus

#|
初期条件：FetchとSpotがそれぞれ73B2の充電ドックにいる．
　　　　　ゴミ袋はゴミ箱にかかっている．
　　　　　(郵便物は機械系事務室にある)
　　　　　ロボットが移動できる場所の組み合わせと移動コスト
ゴール：ゴミ袋がゴミ集積所にある．
　　　　(郵便物が73B2にある)

ロボット間の情報伝達に音声を利用するか通信を利用するかの条件付けとしては
・ロボットどうしの距離(近い時：音声，遠い時：通信)
・近くに人がいるかどうか
のどちらかで記述できそうかと考えています．

人が介入する場面として
・ロボットが動作に失敗しているとき
・ロボットの行き先(指令)を変更したいとき
・ロボットが音声通信に失敗したとき
あたりが考えられる
|#

(ros::roseus "solve_multirobot_task")

(load "package://pddl_planner/src/pddl-result-graph.l")
(load "package://pddl_planner/src/eus-pddl-client.l")

;; domain
(setq *domain* (instance pddl-domain :init :name 'multirobot))
(send *domain* :requirements '(:typing :action-costs))
(send *domain* :types '(robot item place))
;; predicates: 述語
(send *domain* :predicates '((robot-in-place ?robot - robot ?place - place) ;; ?robotが?placeに存在する
                             (item-in-place ?item - item ?place - place) ;; ?itemが?placeに存在する
                             (item-in-robot ?item - item ?robot - robot) ;; ?robotが?itemを持っているか？
                             (can-move ?from ?to - place) ;; ロボットが?toから?fromに移動できるか
                             (can-pick ?robot - robot ?item - item) ;; ロボットが?itemをPickできるか
                             (close-distance ?where - place) ;; ロボット同士の距離
                             (robot-can-not-understand ?robot - robot) ;; ロボットが司令を受け付けない
                             ))
;; 使用する関数の宣言
(send *domain* :functions '((total-cost) ;; 合計の移動コスト
                            (move-cost ?robot - robot ?from - place ?to - place) ;; 各移動にかかるコスト
                            ))

;; making action
(setq *actions*
      (list
       (instance pddl-action :init
                 :name "move"
                 :parameters '((?robot robot) (?from ?to place))
                 :precondition '((robot-in-place ?robot ?from)
                                 (can-move ?from ?to))
                 :effect '((robot-in-place ?robot ?to)
                           (not (robot-in-place ?robot ?from))
                           (increase (total-cost) (move-cost ?robot ?from ?to))
                           ))
       (instance pddl-action :init
                 :name "pick"
                 :parameters '((?robot robot) (?item item) (?where place))
                 :precondition '((robot-in-place ?robot ?where)
                                 (item-in-place ?item ?where)
                                 (can-pick ?robot ?item))
                 :effect '((not (item-in-place ?item ?where))
                           (item-in-robot ?item ?robot)
                           ))
       (instance pddl-action :init
                 :name "place"
                 :parameters '((?robot robot) (?item item) (?where place))
                 :precondition '((robot-in-place ?robot ?where)
                                 (item-in-robot ?item ?robot)
                                 )
                 :effect '((item-in-place ?item ?where)
                           (not (item-in-robot ?item ?robot))
                           ))
       (instance pddl-action :init
                 :name "transfer-by-voice"
                 :parameters '((?from-robot ?to-robot robot) (?item item) (?where place))
                 :precondition '((robot-in-place ?from-robot ?where)
                                 (robot-in-place ?to-robot ?where)
                                 (item-in-robot ?item ?from-robot)
                                 (close-distance ?where)
                                 (not (robot-can-not-understand ?to-robot))
                                 )
                 :effect '((item-in-robot ?item ?to-robot)
                           (not (item-in-robot ?item ?from-robot))
                           ))
       (instance pddl-action :init
                 :name "transfer-by-communication"
                 :parameters '((?from-robot ?to-robot robot) (?item item) (?where place))
                 :precondition '((robot-in-place ?from-robot ?where)
                                 (robot-in-place ?to-robot ?where)
                                 (item-in-robot ?item ?from-robot)
                                 (not (close-distance ?where))
                                 (not (robot-can-not-understand ?to-robot))
                                 )
                 :effect '((item-in-robot ?item ?to-robot)
                           (not (item-in-robot ?item ?from-robot))
                           ))
       ;; https://github.com/jsk-ros-pkg/jsk_planning/blob/bcd1a44f64ccba995edbcc658543f6d381d54d13/task_compiler/samples/failure-recovery-sample/solve-failure-recovery-task.l#L39
       (instance pddl-action :init
                 :name "transfer-by-voice_f" ;; add '_f' to define failure action
                 :parameters '((?from-robot ?to-robot robot) (?item item) (?where place))
                 :precondition '((robot-in-place ?from-robot ?where)
                                 (robot-in-place ?to-robot ?where)
                                 (item-in-robot ?item ?from-robot)
                                 (close-distance ?where)
                                 )
                 :effect '(
                           (robot-can-not-understand ?to-robot) ;; set robot can NOT understand  mode
                           ))
       (instance pddl-action :init
                 :name "transfer-by-human-assist"
                 :parameters '((?from-robot ?to-robot robot) (?item item) (?where place))
                 :precondition '((robot-in-place ?from-robot ?where)
                                 (robot-in-place ?to-robot ?where)
                                 (item-in-robot ?item ?from-robot)
                                 (robot-can-not-understand ?to-robot) ;; when robot can NOT understand
                                 )
                 :effect '((item-in-robot ?item ?to-robot)
                           (not (item-in-robot ?item ?from-robot))
                           (not (robot-can-not-understand ?to-robot)) ;; enable robot to understand
                           ))
       ))

;;add actions to domain
(dolist (action *actions*)
  (send *domain* :add :action action))

;; problem
(setq *problem* (instance pddl-problem :init :name 'tetsudai :domain 'multirobot))

;; 使用する変数の宣言。
(send *problem* :objects
      '(
        ;; item
        (trash         . item)
        ;; robot
        (fetch         . robot)
        (spot          . robot)
        ;; places
        (room-73b2-station  . place)
        (room-73b2-trash    . place)
        (room-trash         . place)
        ))

;; 初期条件
(setq *initial-condition*
      '(;; 物品・ロボットの初期位置
        (item-in-place trash  room-73b2-trash)
        (robot-in-place fetch room-73b2-station)
        (robot-in-place spot  room-73b2-station)
        ;; 移動可能な場所
        (can-move room-73b2-station room-73b2-trash)
        (can-move room-73b2-trash room-73b2-station)
        (can-move room-73b2-station room-trash)
        (can-move room-trash room-73b2-station)
        ;; 把持可能な物品
        (can-pick fetch trash)
        ;; トータル移動コスト
        (= (total-cost) 0)
        ;; fetch移動コスト
        (= (move-cost fetch room-73b2-station room-73b2-trash) 3)
        (= (move-cost fetch room-73b2-trash room-73b2-station) 3)
        (= (move-cost fetch room-73b2-station room-trash) 99999)
        (= (move-cost fetch room-trash room-73b2-station) 99999)
        ;; spot移動コスト
        (= (move-cost spot room-73b2-station room-73b2-trash) 3)
        (= (move-cost spot room-73b2-trash room-73b2-station) 3)
        (= (move-cost spot room-73b2-station room-trash) 100)
        (= (move-cost spot room-trash room-73b2-station) 100)
        ;; ロボット同士の距離
        ;; (close-distance room-73b2-trash)  ;; uncomment to use transfer-by-communication
        (close-distance room-73b2-station)  ;; uncomment to use transfer-by-voice
        ))

;; ゴール条件
(setq *goal-condition*
      '((item-in-place trash room-trash)))

(send *problem* :initial-condition *initial-condition*)
(send *problem* :goal-condition *goal-condition*)

;; solve planning
(send *problem* :metric '(minimize (total-cost)))
(pprint (setq *result* (solve-pddl-planning *domain* *problem*)))
(setq gr (make-graph-from-pddl-results (list *result*) :node-name :pprint))
;;;;
;;;; FAILURE CONTROL USING TASK COMPILER
;;;;
;;;; https://github.com/jsk-ros-pkg/jsk_planning/blame/bcd1a44f64ccba995edbcc658543f6d381d54d13/task_compiler/samples/failure-recovery-sample/solve-failure-recovery-task.l#L113-L114
;;;;
;; swap nodes ( check_open <-> check_open_f ) for making task flow
(setq gr (add-failed-nodes *result* (list 'transfer-by-voice) :graph gr))
;;
(send gr :write-to-pdf "multirobot.pdf")
(when (ros::get-param "~display_graph" "true")
  (piped-fork "xdg-open multirobot.pdf"))

;;;
;;; task planner
;;;
(defun move (robot from to)
  (ros::ros-info "move ~A from ~A ~A" robot from to))
(defun pick (robot item where)
  (ros::ros-info "pick ~A by ~A at ~A" item robot where))
(defun place (robot item where)
  (ros::ros-info "place ~A by ~A at ~A" item robot where))
(defun transfer-by-voice (from-robot to-robot item where)
  (ros::ros-info "transfer ~A from ~A to ~A at ~A by voice" item from-robot to-robot where)
  nil) ;;;; force erro when transfer-by-voice
(defun transfer-by-communication (from-robot to-robot item where)
  (ros::ros-info "transfer ~A from ~A to ~A at ~A by communication" item from-robot to-robot where))
(defun transfer-by-human-assist (from-robot to-robot item where)
  (ros::ros-info "transfer ~A from ~A to ~A at ~A by human" item from-robot to-robot where))

;; convert to smach
(load "package://roseus_smach/src/state-machine-ros.l")
(load "package://roseus_smach/src/state-machine-utils.l")
(load "package://roseus_smach/src/pddl2smach.l")

;; global data is not used (nil)
(exec-smach-with-spin (convert-smach gr) nil :hz 1.0)

;;
(when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
  (ros::exit))
