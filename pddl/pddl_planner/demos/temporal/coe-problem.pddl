(define (problem coe-simple-time-problem)

	(:domain coe-simple-time)

	(:objects
		;; robot, man
		hrp2jsk hrp2v - robot
		satou-prof - man

		;; desk
		kitchen-sink0 - sink
		bar-counter0 - desk
		center-desk0 - desk

		;; cup, petbottle
		cup0 - cup
		petbottle0 - petbottle
		sponge0 - sponge

		;; position
		kitchen-sink0-pos bar-counter0-pos0 bar-counter0-pos1 center-desk0-pos - position)

	(:init
		;; spatial-map
		(connected kitchen-sink0-pos bar-counter0-pos0)
		(connected bar-counter0-pos0 kitchen-sink0-pos)
		(connected center-desk0-pos bar-counter0-pos1)
		(connected bar-counter0-pos1 center-desk0-pos)

		;; robot
		(free-larm hrp2jsk)
		(free-rarm hrp2jsk)
		(free-larm hrp2v)
		(free-rarm hrp2v)
		(at-robot hrp2jsk kitchen-sink0-pos)
		(at-robot hrp2v center-desk0-pos)

		;; man
		(sit satou-prof center-desk0)
	
		;; kitchen-sink0
		(manip-pos kitchen-sink0 kitchen-sink0-pos)

		;; bar-counter0
		(manip-pos bar-counter0 bar-counter0-pos0)
		(manip-pos bar-counter0 bar-counter0-pos1)

		;; center-desk0
		(manip-pos center-desk0 center-desk0-pos)

		;; petbottle
		(on kitchen-sink0 petbottle0)
		(full petbottle0)

		;; cup
		(on kitchen-sink0 cup0)
		;(not (poured cup0))

		;; sponge
		(on kitchen-sink0 sponge0)
	)
	(:goal 
		(and 
			(drinked satou-prof)
			(not (dirty cup0))
			(on kitchen-sink0 cup0)
			(on kitchen-sink0 sponge0)
		))

	(:metric minimize (total-time))
)