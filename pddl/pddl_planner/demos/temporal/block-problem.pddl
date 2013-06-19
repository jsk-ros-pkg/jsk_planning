(define (problem BLOCKS-4-0)

(:domain BLOCKS)

(:objects D B A C - block)

(:INIT 	(CLEAR C) 
	(CLEAR A) 
	(CLEAR B) 
	(CLEAR D) 
	(ON-TABLE C) 
	(ON-TABLE A)
	(ON-TABLE B) 
	(ON-TABLE D) 
	(HANDEMPTY))

(:goal (AND (ON D C) (ON C B) (ON B A)))

(:metric minimize (total-time))
)
