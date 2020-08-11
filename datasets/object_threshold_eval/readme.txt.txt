*****************************************
*					*
*    OBJECT_THRESHOLD_EVAL_ DATASET	*
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
FOLDER:					AUFNAHME:			PROPERTIES:				CONT. NUMB. SCHEME
Buerli					0				Background
					1-15				valid					1-15

Weizenbroetchen_600_6_White_520		0				Background		
					1-12				iO (valid)				16-27
					31-35				cut (non valid)				28-32

Weizenbroetchen_400_6_White_395		0 				background
					1-15				iO (valid)				33-47
					16-31				moderate damages (?non valid)		48-63
					32-40				cut (non valid)				64-72

roggenbroetli				0				Background
					1-15				wrong object (non valid)		73-87

Gipfeli					1-10				wrong object (non valid)		88-97									
					