# Free-For-All.
This are the code which any beginner can use for starting and getting boost in his programming career . 


[1] LANGUAGE:- "Python"

    PROBLEM STATEMENT:- Find whether the number is Armstrong number or not .

    CODE NO :- 1

          n=int(input("enter any number"))
          sum=o
          number=n
          l=len(Str(n))

          while n>0:
          r=n%10
          P=r**l
          sum=sum+P
          n=n//10

          if number==sum:
          print("it is armstrong number")
          else:
          print("not an armstrong number")



[2] LANGUAGE:- "Python"

PROBLEM STATEMENT:- Convert binary to decimal no.

CODE NO :- 2

          n=int(input("enter any number"))
          sum=o
          number=n
          l=len(Str(n))
          mul=0

          while n>0:
          r=n%10
          P=r*(2**mul)
          mul=mul+1
          sum=sum+P
          n=n//10
          print(sum)

