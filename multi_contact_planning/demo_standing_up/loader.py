import pprint
# "/home/francesco/advr-superbuild/external/soap_bar_rrt/multi_contact_planning/planning_data/sigmaList.txt"

def readFromFileStances(path):
    f = open(path, "r")

    stances = []
    while True:
        sigma_size = f.readline()

        if sigma_size == '':
            break

        for j in range(int(sigma_size)):
            # 1. remove \n
            # 2. split into tokes given ' '
            # 3. remove empty '' with filter
            # 4. cast to float with map
            stance = dict(ind=int(f.readline()), ref=dict(pose=map(float, filter(lambda a: a != '', f.readline().strip('\n').split(' '))),
                                                force=map(float, filter(lambda a: a != '', f.readline().strip('\n').split(' '))),
                                                normal=map(float, filter(lambda a: a != '', f.readline().strip('\n').split(' ')))))

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


if __name__ == '__main__':

    stances = readFromFileStances("/home/francesco/advr-superbuild/external/soap_bar_rrt/multi_contact_planning/planning_data/sigmaList.txt")
    q_list = readFromFileConfigs("/home/francesco/advr-superbuild/external/soap_bar_rrt/multi_contact_planning/planning_data/qList.txt")

    pprint.pprint(stances)
    print len(stances)
    print len(q_list)



