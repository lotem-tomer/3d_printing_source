#ifndef WRL_LEXER_HPP
#define WRL_LEXER_HPP

#include <assert.h>
#include <iostream>

#ifndef __FLEX_LEXER_H
#define yyFlexLexer myFlexLexer
#define yywrap mywrap
#include <FlexLexer.h>
#endif

class Wrl_lexer : public yyFlexLexer {
public:
  /*! Constructor */
  Wrl_lexer(std::istream * is = 0, std::ostream * os = 0) :
    yyFlexLexer(is, os), m_lineno(1), isp(is), osp(os)
  {s_instance = this;}

  /*! Destructor */
  virtual ~Wrl_lexer() {}

  static Wrl_lexer * instance(void) {assert(s_instance); return(s_instance);}

  virtual int yylex(void);
  void yyerror(const char * message, int cur_token);

  friend std::istream & operator>>(std::istream &, Wrl_lexer &);
  
private:
  int m_lineno;
  std::istream * isp;  // istream being parsed
  std::ostream * osp;  // ostream being output to

  static Wrl_lexer * s_instance;

  void comment_to_eol(void);
  void comment(void);
};

/*! */
inline std::istream & operator>>(std::istream & is, Wrl_lexer & lexer)
{
  extern int yyparse();

  lexer.isp = &is;
  lexer.m_lineno = 1;
  // lexer.set_debug(1);
  lexer.yyrestart(&is);
  if (yyparse() != 0) is.setstate(std::ios::failbit);
  else is.clear(is.rdstate() & ~(std::ios::failbit | std::ios::badbit));
  return is;
}

#endif
