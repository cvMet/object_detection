*****************************************
*					*
*    Sequence_hard_extended		*
*					*
*****************************************

Aim:
Calculate the # Matches for an object to be considered detected

Method:
Vary the detection threshold (how many keypoints must be detected for an object to be considered detected)
Draw a PR Graph on how many objects were detected correctly.

Notes:
Buerli, roggenbroetli aus threshold_eval dataset. For Gipfeli only cloud data is added.
All clouds are contained in folder sequence (see continuous numbering scheme)

Dataset Structure:
FOLDER:					Count:			PROPERTIES:				CONT. NUMB. SCHEME
Buerli					57				valid					1-72


roggenbroetli				15				wrong object (non valid)		73-87

Gipfeli					10				wrong object (non valid)		88-97

WhiteBottle				15				wrong object (non valid)		98-112	

Mutter					15				wrong object (non valid)		113 - 127		

									
					