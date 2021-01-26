#!/usr/bin/env roseus

#|
使い方
$ roslaunch pddl_planner demo_cooperation_task.launch demo:=office
$ roslaunch pddl_planner demo_cooperation_task.launch demo:=kitchen_car

初期条件：
・fetchは73B2、spotは2号館入り口にいる
・書類は73B2、弁当はキッチンカーにある
・ロボットが移動できる場所は以下の組み合わせのみで、fetch、spotで異なる移動コストが与えられている。
  - 73B2        <-> エレベータ
  - 73B2        <-> 階段
  - エレベータ  <-> 事務室
  - エレベータ  <-> 2号館入り口
  - 階段        <-> 事務室
  - 階段        <-> 2号館入り口
  - 2号館入り口 <-> キッチンカー

終了条件（以下のどちらかをrosparamで選択可能）：
・書類が事務室にあり、かつ、fetchが73B2に戻ってくる
・弁当が73B2にある

その他の条件：
・コストを最小化した経路を計算
・spotはfetchが2号館入り口に居るときのみ移動可能
・ロボット（fetch or spot）は、物品（書類 or 弁当）と同じ場所に居るときのみ、その物品と一緒に移動できる。
|#

(ros::roseus "solve_cooperation_task")

(load "package://pddl_planner/src/pddl-result-graph.l")
(load "package://pddl_planner/src/eus-pddl-client.l")

;; domain
(setq *domain* (instance pddl-domain :init :name 'cooperation))
(send *domain* :requirements '(:typing :action-costs))
(send *domain* :types '(robot item place))
;; predicates: 述語
(send *domain* :predicates '((robot-in-place ?robot - robot ?place - place) ;; ?robotが?placeに存在する
                             (item-in-place ?item - item ?place - place) ;; ?itemが?placeに存在する
                             (can-move ?from ?to - place))) ;; ロボットが?toから?fromに移動できるか
;; 使用する関数の宣言
(send *domain* :functions '((total-cost) ;; 合計の移動コスト
                            (move-cost ?robot - robot ?from - place ?to - place))) ;; 各移動にかかるコスト

;; making action
(setq *actions*
      (list
       (setq *fetch-move* (instance pddl-action :init
                              :name "fetch-move"
                              :parameters '((?from ?to place))
                              :precondition '((robot-in-place fetch ?from)
                                              (can-move ?from ?to))
                              :effect '((robot-in-place fetch ?to)
                                        (not (robot-in-place fetch ?from))
                                        (increase (total-cost) (move-cost fetch ?from ?to)))))
       (setq *spot-move* (instance pddl-action :init
                              :name "spot-move"
                              :parameters '((?from ?to place))
                              :precondition '((robot-in-place spot ?from)
                                              (robot-in-place fetch eng2-entrance)
                                              (can-move ?from ?to))
                              :effect '((robot-in-place spot ?to)
                                        (not (robot-in-place spot ?from))
                                        (increase (total-cost) (move-cost spot ?from ?to)))))
       (setq *fetch-move-with-obj* (instance pddl-action :init
                                       :name "fetch-move-with-obj"
                                       :parameters '((?item item) (?from ?to place))
                                       :precondition '((robot-in-place fetch ?from)
                                                       (item-in-place ?item ?from)
                                                       (can-move ?from ?to))
                                       :effect '((robot-in-place fetch ?to)
                                                 (item-in-place ?item ?to)
                                                 (not (robot-in-place fetch ?from))
                                                 (not (item-in-place ?item ?from))
                                                 (increase (total-cost) (move-cost fetch ?from ?to))
                                                 )))
       (setq *spot-move-with-obj* (instance pddl-action :init
                                       :name "spot-move-with-obj"
                                       :parameters '((?item item) (?from ?to place))
                                       :precondition '((robot-in-place spot ?from)
                                                       (robot-in-place fetch eng2-entrance)
                                                       (item-in-place ?item ?from)
                                                       (can-move ?from ?to))
                                       :effect '((robot-in-place spot ?to)
                                                 (item-in-place ?item ?to)
                                                 (not (robot-in-place spot ?from))
                                                 (not (item-in-place ?item ?from))
                                                 (increase (total-cost) (move-cost spot ?from ?to))
                                                 )))

       ))

;;add actions to domain
(dolist (action *actions*)
  (send *domain* :add :action action))

;; problem
(setq *problem* (instance pddl-problem :init :name 'otsukai :domain 'cooperation))

;; 使用する変数の宣言。
(send *problem* :objects
      '(
        ;; item
        (document      . item)
        (lunch-box     . item)
        ;; robot
        (fetch         . robot)
        (spot          . robot)
        ;; places
        (room-73b2     . place)
        (stairs        . place)
        (elevator      . place)
        (eng2-entrance . place)
        (office        . place)
        (kitchen-car   . place)))

;; 初期条件
(send *problem* :initial-condition
      '(;; 物品・ロボットの初期位置
        (item-in-place lunch-box kitchen-car)
        (item-in-place document room-73b2)
        (robot-in-place fetch room-73b2)
        (robot-in-place spot eng2-entrance)
        ;; 移動可能な場所
        (can-move room-73b2 elevator)
        (can-move room-73b2 stairs)
        (can-move elevator office)
        (can-move elevator eng2-entrance)
        (can-move stairs office)
        (can-move stairs eng2-entrance)
        (can-move eng2-entrance kitchen-car)
        (can-move elevator room-73b2) ;; ここからfromとtoを逆にしている
        (can-move stairs room-73b2)
        (can-move office elevator)
        (can-move eng2-entrance elevator)
        (can-move office stairs)
        (can-move eng2-entrance stairs)
        (can-move kitchen-car eng2-entrance)
        ;; トータル移動コスト
        (= (total-cost) 0)
        ;; fetch移動コスト
        (= (move-cost fetch room-73b2 elevator) 3)
        (= (move-cost fetch room-73b2 stairs) 99999)
        (= (move-cost fetch elevator office) 10)
        (= (move-cost fetch elevator eng2-entrance) 5)
        (= (move-cost fetch stairs office) 99999)
        (= (move-cost fetch stairs eng2-entrance) 1)
        (= (move-cost fetch eng2-entrance kitchen-car) 100)
        (= (move-cost fetch elevator room-73b2) 3) ;; ここからfromとtoを逆にしている
        (= (move-cost fetch stairs room-73b2) 99999)
        (= (move-cost fetch office elevator) 10)
        (= (move-cost fetch eng2-entrance elevator) 5)
        (= (move-cost fetch office stairs) 99999)
        (= (move-cost fetch eng2-entrance stairs) 99999)
        (= (move-cost fetch kitchen-car eng2-entrance) 100)
        ;; spot移動コスト
        (= (move-cost spot room-73b2 elevator) 30)
        (= (move-cost spot room-73b2 stairs) 50)
        (= (move-cost spot elevator office) 100)
        (= (move-cost spot elevator eng2-entrance) 100)
        (= (move-cost spot stairs office) 50)
        (= (move-cost spot stairs eng2-entrance) 50)
        (= (move-cost spot eng2-entrance kitchen-car) 3)
        (= (move-cost spot elevator room-73b2) 30) ;; ここからfromとtoを逆にしている
        (= (move-cost spot stairs room-73b2) 50)
        (= (move-cost spot office elevator) 100)
        (= (move-cost spot eng2-entrance elevator) 100)
        (= (move-cost spot office stairs) 50)
        (= (move-cost spot eng2-entrance stairs) 50)
        (= (move-cost spot kitchen-car eng2-entrance) 3)
        ))

(cond
 ;; 事務室デモ
 ;; 初期条件の追加：最初、書類が73B2にある。
 ;; 終了条件：最後、書類が事務室にあり、かつ、fetchが73B2に戻ってくる
 ((equal (ros::get-param "~demo") "office")
  (setq *initial-condition* (cons '(item-in-place document room-73b2) *initial-condition*))
  (setq *goal-condition*
        '((robot-in-place fetch room-73b2)
          (item-in-place document office))))
 ;; キッチンカーデモ
 ;; 初期条件の追加：最初、お弁当がキッチンカーがある。
 ;; 終了条件：最後、弁当箱が73B2にある。
 ((equal (ros::get-param "~demo") "kitchen_car")
  (setq *initial-condition* (cons '(item-in-place lunch-box kitchen-car) *initial-condition*))
  (setq *goal-condition*
        '((item-in-place lunch-box room-73b2))))
 (t
  (ros::ros-error "Invalid rosparam for ~demo")
  (exit)))

(send *problem* :initial-condition *initial-condition*)
(send *problem* :goal-condition *goal-condition*)

;; solve planning
(send *problem* :metric '(minimize (total-cost)))
(pprint (setq *result* (solve-pddl-planning *domain* *problem*)))
(setq gr (make-graph-from-pddl-results (list *result*) :node-name :pprint))
(send gr :write-to-pdf "cooperation.pdf")
(when (ros::get-param "~display_graph" "true")
  (piped-fork "xdg-open cooperation.pdf"))
(when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
  (ros::exit))
