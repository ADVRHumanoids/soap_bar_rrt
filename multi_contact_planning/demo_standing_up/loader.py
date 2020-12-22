import cartesio_planning.validity_check
import numpy as np

def readFromFileStances(path):
    f = open(path, "r")

    stances = []
    while True:
        sigma_size = f.readline()

        if sigma_size == '':
            break

        stance = []
        for j in range(int(sigma_size)):
            # 1. remove \n
            # 2. split into tokes given ' '
            # 3. remove empty '' with filter
            # 4. cast to float with map
            stance.append(dict(ind=int(f.readline()), ref=dict(pose=map(float, filter(lambda a: a != '', f.readline().strip('\n').split(' '))),
                                                               force=map(float, filter(lambda a: a != '', f.readline().strip('\n').split(' '))),
                                                               normal=map(float, filter(lambda a: a != '', f.readline().strip('\n').split(' '))))))

        stances.append(stance)

    return stances


def readFromFileConfigs(path):
    f = open(path, "r")

    q_list = []
    while True:

        q = f.readline()
        if q == '':
            break

        q = map(float, filter(lambda a: a != '', q.strip('\n').split(' ')))
        q_list.append(q)

    return q_list

def rotation(normal):

    if normal == [0., 0., 1.]:
        theta = [0, 0, 0]
    elif normal == [-1., 0., 0.]:
        theta = [0, -np.pi / 2, 0]
    else:
        raise Exception('wrong normal')

    tx, ty, tz = theta

    Rx = np.array([[1, 0, 0], [0, np.cos(tx), -np.sin(tx)], [0, np.sin(tx), np.cos(tx)]])
    Ry = np.array([[np.cos(ty), 0, np.sin(ty)], [0, 1, 0], [-np.sin(ty), 0, np.cos(ty)]])
    Rz = np.array([[np.cos(tz), -np.sin(tz), 0], [np.sin(tz), np.cos(tz), 0], [0, 0, 1]])

    return np.dot(Rz, np.dot(Rx, Ry))

def checkStability(model, stances, qlist):
    check = []

    for stance, q in zip(stances, qlist):
        active_ind = [ind['ind'] for ind in stance]
        active_links = [model.ctrl_points[j] for j in active_ind]
        model.cs.setContactLinks(active_links)
        get_contact_links = model.cs.getContactLinks()

        normals = [j['ref']['normal'] for j in stance]
        [model.cs.setContactRotationMatrix(k, j) for k, j in zip(active_links, [rotation(elem) for elem in normals])]

        # print 'active links are \n', model.cs.getContactLinks()
        # print 'rotation matrices are: \n', [rotation(elem) for elem in normals]
        # print 'rotation matrices are: \n', [model.cs.getContactFrame(j) for j in active_links]
        # print 'configuration \n', q
        model.model.setJointPosition(q)
        model.model.update()
        model.rspub.publishTransforms('/ci')

        # print 'poses: \n', [model.model.getPose(i) for i in active_links]

        forces = dict(zip(active_links, [np.append(np.array(i['ref']['force']), [0,0,0]) for i in stance]))

        model.cs.setForces(forces)
        # print 'com: \n', model.model.getCOM()
        # print active_links
        # print 'contact:\n', [j['ref']['pose'] for j in stance]
        # print 'forces:\n', [j['ref']['force'] for j in stance]
        # print 'normals:\n', [j['ref']['normal'] for j in stance]
        # print 'forces computed \n', model.cs.getForces()

        # print 'cop',  sum([np.array(j['ref']['pose'])*j['ref']['force'][2]/686.7003 for j in stance])

        check.append(model.state_vc(q))
        print 'active links: ', model.cs.getContactLinks()
        for val in model.cs.getContactLinks():
            print model.cs.getContactFrame(val)

        get_forces = model.cs.getForces()
        # print 'forces computed \n', model.cs.getForces()

    return check

if __name__ == '__main__':

    stances = readFromFileStances("/home/luca/src/MultiDoF-superbuild/external/soap_bar_rrt/multi_contact_planning/planning_data/sigmaList.txt")
    q_list = readFromFileConfigs("/home/luca/src/MultiDoF-superbuild/external/soap_bar_rrt/multi_contact_planning/planning_data/qList.txt")


    print len(stances)
    print len(q_list)

    ee = [0,1,4,5]

    lifted_contact = [x for x in ee if x not in [i['ind'] for i in stances[1]]]

    print lifted_contact



