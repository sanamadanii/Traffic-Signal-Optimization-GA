#==============================importing required modules==============================
import numpy

#====================================algorithm used====================================
def algo(Qa,Qb,Qc,Qd,Qar,Qbr,Qcr,Qdr,Qan,Qbn,Qcn,Qdn):
	
#	def Pressure(Qa):  #assumed linear
#		return (2*Qa+1)
#	U=numpy.zeros((4,4,4))
#	U[0][0]=[1,17,18,19]
#	U[1][1]=[16,1,14,18]
#	U[2][2]=[13,11,1,14]
#	U[3][3]=[9,7,8,1]
#
#	Pressure_Array=Pressure(numpy.array([Qa,Qb,Qc,Qd]))  
#
#	W=numpy.zeros((4,4))
#	#Preparing Weights
#	W[0]=numpy.maximum(((Qa*numpy.array(Qar))/U[0][0]*[0,Pressure(Qa)-Pressure(Qb),Pressure(Qa)-Pressure(Qc),Pressure(Qa)-Pressure(Qd)]),[0,0,0,0])
#	W[1]=numpy.maximum(((Qb*numpy.array(Qbr))/U[1][1]*[Pressure(Qb)-Pressure(Qa),0,Pressure(Qb)-Pressure(Qc),Pressure(Qb)-Pressure(Qd)]),[0,0,0,0])
#	W[2]=numpy.maximum(((Qc*numpy.array(Qcr))/U[2][2]*[Pressure(Qc)-Pressure(Qa),Pressure(Qc)-Pressure(Qb),0,Pressure(Qc)-Pressure(Qd)]),[0,0,0,0])
#	W[3]=numpy.maximum(((Qd*numpy.array(Qdr))/U[3][3]*[Pressure(Qd)-Pressure(Qa),Pressure(Qd)-Pressure(Qb),Pressure(Qd)-Pressure(Qc),0]),[0,0,0,0])
    
#	for i in range(4):
#		W[i][i]=0
#		b=0
#		a=0
#	for i in range (4):
#		if(numpy.sum(W*U[i])>a):
#			a=(numpy.sum(W*U[i]))
#			b=i
#       print(b+1)   
#	return b+1
	W=numpy.zeros((4,4))
	R=numpy.zeros((4,4))
	R[0]=[1,2,3,2]
	R[1]=[2,1,2,3]
	R[2]=[3,2,1,2]
	R[3]=[2,3,2,1]

	W[0]=[Qa-Qar[0]*Qan,Qa-Qar[1]*Qbn,Qa-Qar[2]*Qcn,Qa-Qar[3]*Qdn]
	W[1]=[Qb-Qbr[0]*Qan,Qb-Qbr[1]*Qbn,Qb-Qbr[2]*Qcn,Qb-Qbr[3]*Qdn]
	W[2]=[Qc-Qcr[0]*Qan,Qc-Qcr[1]*Qbn,Qc-Qcr[2]*Qcn,Qc-Qcr[3]*Qdn]
	W[3]=[Qd-Qdr[0]*Qan,Qd-Qdr[1]*Qbn,Qd-Qdr[2]*Qcn,Qd-Qdr[3]*Qdn]
	a=0
	b=0
	for i in range (4):
		if(numpy.sum(W[i]*R[i])>a):
			a=(numpy.sum(W[i]*R[i]))
			b=i
	return b+1

#==============================preparation to call algorithm===========================
def updateJunction(junction):
	
	Q = junction.vehicleVectarr
	Qlen = [len(Q[0]), len(Q[1]), len(Q[2]), len(Q[3])]

	if junction.isFirstPhase:
		junction.isFirstPhase = Qlen.index(max(Qlen)) + 1
	else:
		green = junction.green
		tempList = [1, 2, 3, 4]
		tempList.remove(green)
		Qij = []
		for k in tempList:
			Qij += [len([i for i in Q[green-1] for j in Q[k-1] if i == j])]
		#print(Qij)
		Qba_old = junction.Qba
		Qab_old = junction.Qab
		Qca_old = junction.Qca
		Qac_old = junction.Qac
		Qbc_old = junction.Qbc
		Qad_old = junction.Qad
		Qcb_old = junction.Qcb
		Qda_old = junction.Qda
		Qbd_old = junction.Qbd
		Qdb_old = junction.Qdb
		Qcd_old = junction.Qcd
		Qdc_old = junction.Qdc

		if green == 1:
			Qlen[1] -= Qij[0]
			Qlen[2] -= Qij[1]
			Qlen[3] -= Qij[2]
			Qlen[0] -= (Qij[0] + Qij[1] + Qij[2])
			if junction.QaNum != 0:
				junction.Qab = (float(Qij[0])/((junction.QaNum)))*Qlen[0]
				junction.Qac = (float(Qij[1])/((junction.QaNum)))*Qlen[0]
				junction.Qad = (float(Qij[2])/((junction.QaNum)))*Qlen[0]
		
		elif green == 2:
			Qlen[0] -= Qij[0]
			Qlen[2] -= Qij[1]
			Qlen[3] -= Qij[2]
			Qlen[1] -= (Qij[0] + Qij[1] + Qij[2])
			if junction.QbNum != 0:
				junction.Qba = (float(Qij[0])/((junction.QbNum)))*Qlen[1]
				junction.Qbc = (float(Qij[1])/((junction.QbNum)))*Qlen[1]
				junction.Qbd = (float(Qij[2])/((junction.QbNum)))*Qlen[1]	
		elif green == 3:
			Qlen[0] -= Qij[0]
			Qlen[1] -= Qij[1]
			Qlen[3] -= Qij[2]
			Qlen[2] -= (Qij[0] + Qij[1] + Qij[2])
			if junction.QcNum != 0:
				junction.Qca = (float(Qij[0])/((junction.QcNum)))*Qlen[2]
				junction.Qcb = (float(Qij[1])/((junction.QcNum)))*Qlen[2]
				junction.Qcd = (float(Qij[2])/((junction.QcNum)))*Qlen[2]
		else:
			Qlen[0] -= Qij[0]
			Qlen[1] -= Qij[1]
			Qlen[2] -= Qij[2]
			Qlen[3] -= (Qij[0] + Qij[1] + Qij[2])
			if junction.QdNum != 0:
				junction.Qda = (float(Qij[0])/((junction.QdNum)))*Qlen[3]
				junction.Qdb = (float(Qij[1])/((junction.QdNum)))*Qlen[3]
				junction.Qdc = (float(Qij[2])/((junction.QdNum)))*Qlen[3]

	QlenOld = [junction.QaNum, junction.QbNum, junction.QcNum, junction.QdNum]
	junction.QaNum,  junction.QbNum, junction.QcNum, junction.QdNum = Qlen[0], Qlen[1], Qlen[2], Qlen[3]

	if junction.isFirstPhase:
		junction.isFirstPhase = False
		return junction
	#junction update done
	#preparing to call algo:
	Qar = Qar_old = Qbr = Qbr_old = Qcr = Qcr_old = Qdr = Qdr_old = []
	Qlen = numpy.array(Qlen)
	if Qlen[0] != 0:
		Qar = numpy.insert((numpy.array([junction.Qab, junction.Qac, junction.Qad]))/float(Qlen[0]), 0, 0)
	if QlenOld[0] != 0:
		Qar_old = numpy.insert((numpy.array([Qab_old, Qac_old, Qad_old]))/float(QlenOld[0]), 0, 0)
	if not bool(list(Qar_old)):
		Qar_old = Qar
	if not bool(list(Qar)):
		Qar = Qar_old
	if (not bool(list(Qar_old))) and (not bool(list(Qar))):
		Qar = Qar_old = numpy.array([0,1.0/3,1.0/3, 1.0/3])

	if Qlen[1] != 0:
		Qbr = numpy.insert((numpy.array([junction.Qba, junction.Qbc, junction.Qbd]))/float(Qlen[1]), 1, 0)
	if QlenOld[1] != 0:
		Qbr_old = numpy.insert((numpy.array([Qba_old, Qbc_old, Qbd_old]))/float(QlenOld[1]), 1, 0)
	if not bool(list(Qbr_old)):
		Qbr_old = Qbr
	if not bool(list(Qbr)) == 0:
		Qbr = Qbr_old
	if (not bool(list(Qbr_old))) and (not bool(list(Qbr))):
		Qbr = Qbr_old = numpy.array([1.0/3, 0,1.0/3, 1.0/3])

	if Qlen[2] != 0:
		Qcr = numpy.insert((numpy.array([junction.Qca, junction.Qcb, junction.Qcd]))/float(Qlen[2]), 2, 0)
	if QlenOld[2] != 0:
		Qcr_old = numpy.insert((numpy.array([Qca_old, Qcb_old, Qcd_old]))/float(QlenOld[2]), 2, 0)
	if not bool(list(Qcr_old)):
		Qcr_old = Qcr
	if not bool(list(Qcr)):
		Qcr = Qcr_old
	if (not bool(list(Qcr_old))) and (not bool(list(Qcr))):
		Qcr = Qcr_old = numpy.array([1.0/3, 1.0/3, 0, 1.0/3])

	if Qlen[3] != 0:
		Qdr = numpy.insert((numpy.array([junction.Qda, junction.Qdb, junction.Qdc]))/float(Qlen[3]), 3, 0)
	if QlenOld[3] != 0:
		Qdr_old = numpy.insert((numpy.array([Qda_old, Qdb_old, Qdc_old]))/float(QlenOld[3]), 3, 0)
	if not bool(list(Qdr_old)):
		Qdr_old = Qdr
	if not bool(list(Qdr)):
		Qdr = Qdr_old
	if (not bool(list(Qdr_old))) and (not bool(list(Qdr))):
		Qdr = Qdr_old = numpy.array([1.0/3,1.0/3, 1.0/3, 0])
	#print(Qar_old,Qbr_old,Qcr_old,Qdr_old,Qar,Qbr,Qcr,Qdr)
	
	if numpy.sum(Qar+Qar_old)==0:
		Qar=[0,1.0/3,1.0/3,1.0/3]
	else :
	  if numpy.sum(Qar)==0 :
	   Qar = Qar_old
	  if numpy.sum(Qar_old!=0) and numpy.sum(Qar!=0) :
           Qar = ((2*Qar + Qar_old)/(3.0))

	if numpy.sum(Qbr)==0 and numpy.sum(Qbr_old)==0:
		Qbr=[1.0/3,0,1.0/3,1.0/3]
	else :
	  if numpy.sum(Qbr)==0 :
	   Qbr = Qbr_old
	  if numpy.sum(Qbr_old!=0) and numpy.sum(Qbr!=0) :
           Qbr = ((2*Qbr + Qbr_old)/(3.0))

	if numpy.sum(Qcr+Qcr_old)==0:
		Qcr=[1.0/3,1.0/3,0,1.0/3]
	else :
	  if numpy.sum(Qcr)==0 :
	   Qcr = Qcr_old
	  if numpy.sum(Qcr_old!=0) and numpy.sum(Qcr!=0) :
           Qcr = ((2*Qcr + Qcr_old)/(3.0))

	if numpy.sum(Qdr+Qdr_old)==0:
		Qdr=[1.0/3,1.0/3,1.0/3,0]
	else :
	  if numpy.sum(Qdr)==0 :
	   Qdr = Qdr_old
	  if numpy.sum(Qdr_old!=0) and numpy.sum(Qdr!=0) :
           Qdr = ((2*Qdr + Qdr_old)/(3.0))


	#print(Qar,Qbr,Qcr,Qdr,Qij)
	Qan, Qbn, Qcn, Qdn = 0, 0, 0, 0
	for neighbour in junction.neighbours:
		connection = neighbour['connection'][0]
		data = neighbour['data']
		if connection == 'a':
			Qan = data
		elif connection == 'b':
			Qbn = data
		elif connection == 'c':
			Qcn = data
		elif connection == 'd':
			Qdn == data
	#calling algo:
	junction.green = algo(Qlen[0], Qlen[1], Qlen[2], Qlen[3], Qar, Qbr, Qcr, Qdr, Qan, Qbn, Qcn, Qdn)

	return junction
