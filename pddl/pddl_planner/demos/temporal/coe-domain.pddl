(define (domain coe-simple-time)

	(:requirements :typing :durative-actions)

	(:types single-manip-object area position - object
                robot - robot
                man - man
		cup petbottle sponge - single-manip-object
		sink desk - area
		position - position)

	(:predicates 
		;; spatial-map
		(connected ?from ?to - position)

		;; robot
		(free-larm ?robot)
		(free-rarm ?robot)
		(at-robot ?robot - robot ?pos - position)
		(grasp-larm ?robot - robot ?single-manip-obj - single-manip-object)
		(grasp-rarm ?robot - robot ?single-manip-obj - single-manip-object)

		;; man
		(sit ?man - man ?desk - area)
		(drinked ?man - man)

		;; manipulation-pos
		(manip-pos ?desk - area ?pos - position)						

		;; desk		
		(on ?desk - area ?single-manip-obj - single-manip-object)

		;; petbottle
		(full ?petbottle - petbottle)

		;; cup
		(poured ?cup - cup)
		(dirty ?cup - cup)
	)

	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;;
	;; Robot : Transit/Transfer
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	;; Transit : [?robot] can move from [?from] to [?to].
	(:durative-action transit
		:parameters (?robot - robot ?from - position ?to - position)
	        :duration (= ?duration 1)
		:condition (and 
				(at start (free-larm ?robot))
				(at start (free-rarm ?robot))
				(at start (at-robot ?robot ?from))
				(over all (connected ?from ?to)))
		:effect (and 
				(at start (not (at-robot ?robot ?from)))
				(at end (at-robot ?robot ?to))))

	;; Transfer : [?robot] can move from [?from] to [?to].
	(:durative-action transfer-larm
		:parameters (?robot - robot ?from - position ?to - position)
		:duration (= ?duration 1)
		:condition (and 
				(over all (not (free-larm ?robot)))
				(over all (free-rarm ?robot))
				(at start (at-robot ?robot ?from))
				(over all (connected ?from ?to)))
		:effect (and 
				(at start (not (at-robot ?robot ?from)))
				(at end (at-robot ?robot ?to))))

	;; Transfer : [?robot] can move from [?from] to [?to].
	(:durative-action transfer-rarm
		:parameters (?robot - robot ?from - position ?to - position)
		:duration (= ?duration 1)
		:condition (and 
				(over all (free-larm ?robot))
				(over all (not (free-rarm ?robot)))
				(at start (at-robot ?robot ?from))
				(over all (connected ?from ?to)))
		:effect (and 
				(at start (not (at-robot ?robot ?from)))
				(at end (at-robot ?robot ?to))))

	;; Transfer : [?robot] can move from [?from] to [?to].
	(:durative-action transfer-arms
		:parameters (?robot - robot ?from - position ?to - position)
		:duration (= ?duration 1)
		:condition (and 				
				(over all (not (free-larm ?robot)))
				(over all (not (free-rarm ?robot)))
				(at start (at-robot ?robot ?from))
				(over all (connected ?from ?to)))
		:effect (and 
				(at start (not (at-robot ?robot ?from)))
				(at end (at-robot ?robot ?to))))

	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;;
	;; Robot : Pick
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	;; Pick-Larm : [?robot] (which is at [?pos]) can pick [?single-manip-obj] on [?desk].
	(:durative-action pick-larm
		:parameters (?robot - robot ?single-manip-obj - single-manip-object  ?desk - area ?pos - position)
		:duration (= ?duration 1)
		:condition (and 
				(at start (free-larm ?robot))
				(over all (at-robot ?robot ?pos))
				(over all (manip-pos ?desk ?pos))
				(at start (on ?desk ?single-manip-obj)))
		:effect (and 
			   (at start (not (free-larm ?robot)))
			   (at start (grasp-larm ?robot ?single-manip-obj))
			   (at start (not (on ?desk ?single-manip-obj)))))

	;; Pick-Rarm : [?robot] (which is at [?pos]) can pick [?single-manip-obj] on [?desk].
	(:durative-action pick-rarm
		:parameters (?robot - robot ?single-manip-obj - single-manip-object  ?desk - area ?pos - position)
		:duration (= ?duration 1)
		:condition (and 
				(at start (free-rarm ?robot))
				(over all (at-robot ?robot ?pos))
				(over all (manip-pos ?desk ?pos))
				(at start (on ?desk ?single-manip-obj)))
		:effect (and 
			   (at start (not (free-rarm ?robot)))
			   (at start (grasp-rarm ?robot ?single-manip-obj))
			   (at start (not (on ?desk ?single-manip-obj)))))

	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;;
	;; Robot : Unpick
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	;; Unpick-Larm : [?robot] (which is at [?pos]) can unpick [?single-manip-obj] on [?desk].
	(:durative-action unpick-larm
		:parameters (?robot - robot ?single-manip-obj - single-manip-object ?desk - area ?pos - position)
		:duration (= ?duration 1)
	 	:condition (and
				(over all (not (free-larm ?robot)))
				(over all (grasp-larm ?robot ?single-manip-obj))
				(over all (at-robot ?robot ?pos))
				(over all (manip-pos ?desk ?pos))
			 	(over all (not (on ?desk ?single-manip-obj))))
		:effect (and 
				(at end (free-larm ?robot))
				(at end (not (grasp-larm ?robot ?single-manip-obj)))
				(at end (on ?desk ?single-manip-obj))))

	;; Unpick-Rarm : [?robot] (which is at [?pos]) can unpick [?single-manip-obj] on [?desk].
	(:durative-action unpick-rarm
		:parameters (?robot - robot ?single-manip-obj - single-manip-object ?desk - area ?pos - position)
		:duration (= ?duration 1)
	 	:condition (and
				(over all (not (free-rarm ?robot)))
				(over all (grasp-rarm ?robot ?single-manip-obj))
				(over all (at-robot ?robot ?pos))
				(over all (manip-pos ?desk ?pos))
			 	(over all (not (on ?desk ?single-manip-obj))))
		:effect (and 
				(at end (free-rarm ?robot))
				(at end (not (grasp-rarm ?robot ?single-manip-obj)))
				(at end (on ?desk ?single-manip-obj))))

	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;;
	;; Robot : Pour
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	;; Pour-LRarm : [?robot] at [?pos] can pour larm[?petbottle] into rarm[?cup].
	(:durative-action pour-lrarm
		:parameters (?robot - robot 
			     ?petbottle - petbottle ?cup - cup)
		:duration (= ?duration 1)
		:condition (and 
				(over all (not (free-larm ?robot)))
				(over all (not (free-rarm ?robot)))
				(over all (grasp-larm ?robot ?petbottle))
				(over all (grasp-rarm ?robot ?cup))
				(at start (not (poured ?cup)))
				(at start (full ?petbottle)))
		:effect (and 
				(at end (poured ?cup))
				(at end (not (full ?petbottle)))))

	;; Pour-RLarm : [?robot] at [?pos] can pour rarm[?petbottle] into larm[?cup].
	(:durative-action pour-rlarm
		:parameters (?robot - robot 
			     ?petbottle - petbottle ?cup - cup)
		:duration (= ?duration 1)
		:condition (and 
				(over all (not (free-larm ?robot)))
				(over all (not (free-rarm ?robot)))
				(over all (grasp-larm ?robot ?cup))
				(over all (grasp-rarm ?robot ?petbottle))
				(at start (not (poured ?cup)))
				(at start (full ?petbottle)))
		:effect (and 
				(at end (poured ?cup))
				(at end (not (full ?petbottle)))))

	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;;
	;; Robot : Wash
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	;; Wash : [?robot] can wash [?cup] on [?sink].
	(:durative-action wash-lrarm
		:parameters (?robot - robot ?sponge - sponge ?cup - cup ?sink - sink ?pos - position)
		:duration (= ?duration 1)
		:condition (and
				(over all (not (free-larm ?robot)))
				(over all (not (free-rarm ?robot)))
				(over all (grasp-larm ?robot ?sponge))
				(over all (grasp-rarm ?robot ?cup))
				(over all (at-robot ?robot ?pos))
				(over all (manip-pos ?sink ?pos))
				(at start (dirty ?cup)))
		:effect (and
				(at end (not (dirty ?cup)))))

	;; Wash : [?robot] can wash [?cup] on [?sink].
	(:durative-action wash-rlarm
		:parameters (?robot - robot ?sponge - sponge ?cup - cup ?sink - sink ?pos - position)
		:duration (= ?duration 1)
		:condition (and
				(over all (not (free-larm ?robot)))
				(over all (not (free-rarm ?robot)))
				(over all (grasp-larm ?robot ?cup))
				(over all (grasp-rarm ?robot ?sponge))
				(over all (at-robot ?robot ?pos))
				(over all (manip-pos ?sink ?pos))
				(at start (dirty ?cup)))
		:effect (and
				(at end (not (dirty ?cup)))))

	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;;
	;; Man : Drink
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 
	;; Drink : [?man] can drink [?cup] on [?desk].
	(:durative-action drink
		:parameters (?man - man ?cup - cup ?desk - desk)
		:duration (= ?duration 1)
		:condition (and 
				(over all (sit ?man ?desk))
				(over all  (on ?desk ?cup))
				(at start (poured ?cup))
				(at start (not (dirty ?cup))))
		:effect (and 
				(at end (not (poured ?cup)))
				(at end (dirty ?cup))
				(at end (drinked ?man))))
)