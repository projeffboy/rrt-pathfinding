import time
import random
import drawSample
import math
import _tkinter
import sys
import imageToRects
import utils

#display = drawSample.SelectRect(imfile=im2Small,keepcontrol=0,quitLabel="")
args = utils.get_args()
visualize = utils.get_args()
drawInterval = 100 # 10 is good for normal real-time drawing

prompt_before_next=1  # ask before re-running sonce solved
SMALLSTEP = args.step_size # what our "local planner" can handle.
map_size,obstacles = imageToRects.imageToRects(args.world)
#Note the obstacles are the two corner points of a rectangle
#Each obstacle is (x1,y1), (x2,y2), making for 4 points
XMAX = map_size[0]
YMAX = map_size[1]

G = [  [ 0 ]  , [] ]   # nodes, edges
vertices = [ [args.start_pos_x, args.start_pos_y], [args.start_pos_x, args.start_pos_y + 10]]

# goal/target
tx = args.target_pos_x
ty = args.target_pos_y

# start
sigmax_for_randgen = XMAX/2.0
sigmay_for_randgen = YMAX/2.0
nodes=0
edges=1

def redraw(canvas):
    canvas.clear()
    canvas.markit( tx, ty, r=SMALLSTEP )
    drawGraph(G, canvas)
    for o in obstacles: canvas.showRect(o, outline='blue', fill='blue')
    canvas.delete("debug")


def drawGraph(G, canvas):
    global vertices,nodes,edges
    if not visualize: return
    for i in G[edges]:
       canvas.polyline(  [vertices[i[0]], vertices[i[1]] ]  )


def genPoint():
    if args.rrt_sampling_policy == "uniform":
        # Uniform distribution
        x = random.random()*XMAX
        y = random.random()*YMAX
    elif args.rrt_sampling_policy == "gaussian":
        # Gaussian with mean at the goal
        x = random.gauss(tx, sigmax_for_randgen)
        y = random.gauss(ty, sigmay_for_randgen)
    elif args.rrt_sampling_policy == "triangular":
        # Gaussian with mean at the goal
        x = random.triangular(0, tx, XMAX)
        y = random.triangular(0, ty, YMAX)
    else:
        print "Not yet implemented"
        quit(1)

    bad = 1
    while bad:
        bad = 0
        if args.rrt_sampling_policy == "uniform":
            # Uniform distribution
            x = random.random()*XMAX
            y = random.random()*YMAX
        elif args.rrt_sampling_policy == "gaussian":
            # Gaussian with mean at the goal
            x = random.gauss(tx, sigmax_for_randgen)
            y = random.gauss(ty, sigmay_for_randgen)
        elif args.rrt_sampling_policy == "triangular":
            # Gaussian with mean at the goal
            x = random.triangular(0, tx, XMAX)
            y = random.triangular(0, ty, YMAX)
        else:
            print "Not yet implemented"
            quit(1)
        # range check for gaussian
        if x<0: bad = 1
        if y<0: bad = 1
        if x>XMAX: bad = 1
        if y>YMAX: bad = 1
    return [x,y]

def returnParent(k, canvas, G):
    """ Return parent note for input node k. """
    for e in G[edges]:
        if e[1]==k:
            canvas.polyline(  [vertices[e[0]], vertices[e[1]] ], style=3  )
            return e[0]

def genvertex():
    vertices.append( genPoint() )
    return len(vertices)-1

def pointToVertex(p):
    vertices.append( p )
    return len(vertices)-1

def pickvertex():
    return random.choice( range(len(vertices) ))

def lineFromPoints(p1,p2): # vector starts from p1
    #TODO
    vector = [p2[0] - p1[0], p2[1] - p1[1]]
    return vector

#TODO (INFORMAL)
def pointpointSqDist(p1, p2):
    x = p2[0] - p1[0]
    y = p2[1] - p1[1]
    sqDist = x ** 2 + y ** 2
    return sqDist

def pointPointDistance(p1,p2):
    #TODO
    sqDist = pointpointSqDist(p1, p2)
    return math.sqrt(sqDist)

# x_nearest <- Nearest(G = (V, E), x_rand)
def closestPointToPoint(G,p2):
    #TODO
    min_sq_dist = float('inf')
    parent = 0
    for i in G[0]:
        sq_dist = pointpointSqDist(vertices[i], p2)
        if sq_dist < min_sq_dist:
            min_sq_dist = sq_dist
            parent = i
    #return vertex index
    return parent

def signedArea(l1, l2):
    return l1[0] * l2[1] - l2[0] * l1[1]

def notSameSgn(num1, num2):
    return ((num1 <= 0 and num2 >= 0) \
        or (num1 >= 0 and num2 <= 0)) \
        and not (num1 == 0 and num2 == 0)

def lineHitsRect(p1,p2,r):
    #TODO
    x1, y1, x2, y2 = r # rectangle corner points
    sides = [
        [[x1, y1], [x2, y1]],
        [[x1, y2], [x2, y2]],
        [[x1, y1], [x1, y2]],
        [[x2, y1], [x2, y2]],
    ]
    
    for side in sides:
        l1 = lineFromPoints(p1, p2)
        l2 = lineFromPoints(p2, side[0])
        l3 = lineFromPoints(p2, side[1])
        turn1 = signedArea(l1, l2)
        turn2 = signedArea(l1, l3)
        l4 = lineFromPoints(side[0], side[1])
        l5 = lineFromPoints(side[1], p1)
        l6 = lineFromPoints(side[1], p2)
        turn3 = signedArea(l4, l5)
        turn4 = signedArea(l4, l6)

        if notSameSgn(turn1, turn2) and notSameSgn(turn3, turn4):
            return True

    return False

def inRect(p,rect,dilation):
    """ Return 1 in p is inside rect, dilated by dilation (for edge cases). """
    #TODO
    # x1 <= x <= x2, y1 <= y <= y2
    if rect[0] - dilation <= p[0] \
    and p[0] <= rect[2] + dilation \
    and rect[1] - dilation <= p[1] \
    and p[1] <= rect[3] + dilation:
        return 1
    else:
        return 0

def rrt_search(G, tx, ty, canvas):
    #TODO
    #Fill this function as needed to work ...


    global sigmax_for_randgen, sigmay_for_randgen
    n=0
    nsteps=0
    rrt_iterations = 0 # number of rrt iterations
    while 1:
        #TODO (INFORMAL)
        rrt_iterations += 1

        p = genPoint() # x_rand <- SampleFree
        v = closestPointToPoint(G,p)
        #TODO (INFORMAL) x_new <- Steer(x_nearest, x_rand)
        vector = lineFromPoints(vertices[v], p)
        magnitude = math.sqrt(vector[0] ** 2 + vector[1] ** 2)
        unit_vector = [vector[0] / magnitude, vector[1] / magnitude]
        step_vector = [SMALLSTEP * unit_vector[0], SMALLSTEP * unit_vector[1]]
        p = [sum(i) for i in zip(vertices[v], step_vector)]

        if visualize:
            # if nsteps%500 == 0: redraw()  # erase generated points now and then or it gets too cluttered
            n=n+1
            if n>10:
                canvas.events()
                n=0

        #TODO (INFORMAL)
        restart = False

        for o in obstacles:
            #if inRect(p,o,1):
            if lineHitsRect(vertices[v],p,o) or inRect(p,o,1):
                # print "TODO"
                restart = True
                break
                #... reject
        
        #TODO (INFORMAL)
        if restart: continue

        k = pointToVertex( p )   # is the new vertex ID
        G[nodes].append(k)
        G[edges].append( (v,k) )
        if visualize:
            canvas.polyline(  [vertices[v], vertices[k] ]  )

        #TODO (INFORMAL)
        nsteps += 1

        if pointPointDistance( p, [tx,ty] ) < SMALLSTEP:
            print "Target achieved.", nsteps, "nodes in entire tree"
            if visualize:
                t = pointToVertex([tx, ty])  # is the new vertex ID
                G[edges].append((k, t))
                if visualize:
                    canvas.polyline([p, vertices[t]], 1)
                # while 1:
                #     # backtrace and show the solution ...
                #     canvas.events()
                nsteps = 0
                totaldist = 0
                while 1:
                    oldp = vertices[k]  # remember point to compute distance
                    k = returnParent(k, canvas, G)  # follow links back to root.
                    canvas.events()
                    if k <= 1: break  # have we arrived?
                    nsteps = nsteps + 1  # count steps
                    totaldist = totaldist + pointPointDistance(vertices[k], oldp)  # sum lengths
                print "Path length", totaldist, "using", nsteps, "nodes."
                #TODO (INFORMAL)
                print "Number of iterations: ", rrt_iterations

                global prompt_before_next
                if prompt_before_next:
                    canvas.events()
                    print "More [c,q,g,Y]>",
                    d = sys.stdin.readline().strip().lstrip()
                    print "[" + d + "]"
                    if d == "c": canvas.delete()
                    if d == "q": return
                    if d == "g": prompt_before_next = 0
                break

def main():
    #seed
    random.seed(args.seed)
    if visualize:
        canvas = drawSample.SelectRect(xmin=0,ymin=0,xmax=XMAX ,ymax=YMAX, nrects=0, keepcontrol=0)#, rescale=800/1800.)
        for o in obstacles: canvas.showRect(o, outline='red', fill='blue')
    while 1:
        # graph G
        G = [  [ 0 ]  , [] ]   # nodes, edges
        redraw(canvas)
        G[edges].append( (0,1) )
        G[nodes].append(1)
        if visualize: canvas.markit( tx, ty, r=SMALLSTEP )

        drawGraph(G, canvas)
        rrt_search(G, tx, ty, canvas)

    if visualize:
        canvas.mainloop()

if __name__ == '__main__':
    main()