import methods

def derivative(x,y):
    return -0.5*(y**0.5)
def real_y(x):
    y=(-0.25*x+3)**2
    return y 
CN=methods.CrankNicolson(derivative,real_y,0.01,(0,9),None,6)

CN.plot()
CN.plot_err()